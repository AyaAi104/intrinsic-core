// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "intrinsic/icon/control/algorithms/cartesian_impedance_commons.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "Eigen/Eigenvalues"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/cartesian_admittance_action.pb.h"
#include "intrinsic/icon/actions/cartesian_admittance_info.h"
#include "intrinsic/icon/actions/cartesian_impedance_parameters.h"
#include "intrinsic/icon/control/algorithms/compute_critical_damping.h"
#include "intrinsic/icon/proto/cart_space_conversion.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto/vector3.pb.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/math/twist.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {
namespace cartesian_impedance {

constexpr double kMinVectorNorm = 1e-5;

void CartesianErrorState::Reset() {
  pose.setConstant(std::numeric_limits<double>::max());
  twist.setConstant(std::numeric_limits<double>::max());
  wrench.setConstant(std::numeric_limits<double>::max());
  pose_integrated.setZero();
}

absl::Status Preprocess(CartesianTarget& params) {
  INTRINSIC_ASSERT_NON_REALTIME();
  // Validate user-defined virtual cartesian inertia.
  if (Eigen::EigenSolver<eigenmath::Matrix6d>(params.virtual_cartesian_inertia)
          .eigenvalues()
          .real()
          .array()
          .minCoeff() < kNumericPrecision) {
    return absl::FailedPreconditionError(
        "The virtual cartesian inertia matrix must be strictly positive "
        "definite. Eigenvalues too small.");
  }
  if (!params.virtual_cartesian_inertia.isApprox(
          params.virtual_cartesian_inertia.transpose(), kNumericPrecision)) {
    return absl::FailedPreconditionError(
        "The virtual cartesian inertia matrix must be symmetric.");
  }

  // Validate user-defined cartesian stiffness.
  if (Eigen::EigenSolver<eigenmath::Matrix6d>(params.cartesian_stiffness)
          .eigenvalues()
          .real()
          .array()
          .minCoeff() < -kNumericPrecision) {
    return absl::FailedPreconditionError(
        "The stiffness matrix must be positive semi-definite.");
  }
  if (!params.cartesian_stiffness.isApprox(
          params.cartesian_stiffness.transpose(), kNumericPrecision)) {
    return absl::FailedPreconditionError(
        "The stiffness matrix must be symmetric.");
  }

  // Check and assign user-defined cartesian damping or compute
  // automatically-derived damping matrix.
  if (params.cartesian_damping.has_value()) {
    const auto damping_eval = Eigen::EigenSolver<eigenmath::Matrix6d>(
                                  params.cartesian_damping.value())
                                  .eigenvalues();
    if (damping_eval.real().array().minCoeff() < 0.0) {
      return absl::FailedPreconditionError(
          "The damping matrix must be positive semi-definite.");
    }
    if (damping_eval.real().array().maxCoeff() < kNumericPrecision) {
      return absl::FailedPreconditionError(
          "The damping matrix eigenvalues are all close to zero, instability "
          "likely. Terminating.");
    }
    if (!params.cartesian_damping->isApprox(
            params.cartesian_damping->transpose(), kNumericPrecision)) {
      return absl::FailedPreconditionError(
          "The damping matrix must be symmetric.");
    }
  } else {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        params.cartesian_damping,
        ComputeCriticalDamping(params.virtual_cartesian_inertia,
                               params.cartesian_stiffness));
  }

  // Check and assign user-defined wrench selection matrix in case it differs
  // from Identity.
  if (!params.wrench_selection_matrix.isApprox(
          eigenmath::Matrix6d::Identity())) {
    eigenmath::Matrix6d wrench_selection_matrix_squared =
        params.wrench_selection_matrix * params.wrench_selection_matrix;

    // The properties P = P² and P = P.transpose() are sufficient for a square
    // matrix to be an orthogonal projection matrix.
    if (!wrench_selection_matrix_squared.isApprox(
            params.wrench_selection_matrix, kNumericPrecision)) {
      return absl::FailedPreconditionError(
          "The wrench selection matrix is not an orthogonal projection matrix, "
          "the property P = P² does not hold.");
    }
    if (!params.wrench_selection_matrix.isApprox(
            params.wrench_selection_matrix.transpose(), kNumericPrecision)) {
      return absl::FailedPreconditionError(
          "The wrench selection matrix is not an orthogonal projection matrix, "
          "the property P = P.transpose() does not hold.");
    }
  }
  return absl::OkStatus();
}

absl::Status Preprocess(NullspaceTarget& params) {
  INTRINSIC_ASSERT_NON_REALTIME();
  // Check and assign the user-provided nullspace stiffness.
  params.nullspace_stiffness =
      params.nullspace_stiffness.value_or(kDefaultNullspaceStiffness);
  if (params.nullspace_stiffness < 0.0) {
    return absl::FailedPreconditionError(
        "The nullspace stiffness must greater or equal to zero.");
  }

  // Check, assign or alternatively derive automatically derive the nullspace
  // damping.
  params.nullspace_damping = params.nullspace_damping.value_or(
      2.0 * std::sqrt(*params.nullspace_stiffness));
  if (params.nullspace_damping < 0.0) {
    return absl::FailedPreconditionError(
        "The nullspace damping must be greater than or equal to zero.");
  }
  return absl::OkStatus();
}

absl::Status Preprocess(AlgorithmConfiguration& params) {
  INTRINSIC_ASSERT_NON_REALTIME();
  // Validate user-defined deadband wrench.
  if (params.sensed_wrench_deadband.minCoeff() < 0.0) {
    return absl::FailedPreconditionError("Deadband contains negative numbers.");
  }

  if (params.jacobian_pinv_damping < 0.0) {
    return absl::FailedPreconditionError(
        "The Jacobian pseudoinverse damping must not be smaller than zero.");
  }

  if (params.pose_error_integrator_gain.minCoeff() < 0.0) {
    return absl::FailedPreconditionError(
        "The pose error integrator gain must be greater or equal than zero.");
  }

  if (params.pose_error_integrator_bound.minCoeff() < 0.0) {
    return absl::FailedPreconditionError(
        "The pose error integrator upper bound must be greater or equal than "
        "zero.");
  }

  if (params.post_sensor_dynamic_load_multiplier < 0.0 ||
      params.post_sensor_dynamic_load_multiplier > 1.0) {
    return absl::FailedPreconditionError(
        absl::StrCat("The multipier for post-sensor dynamics compensation must "
                     "be in the interval [0.0, 1.0], got ",
                     params.post_sensor_dynamic_load_multiplier));
  }
  return absl::OkStatus();
}

absl::Status Preprocess(StateVariableConfiguration& params) {
  INTRINSIC_ASSERT_NON_REALTIME();
  if (params.position_error_threshold < kNumericPrecision) {
    return absl::FailedPreconditionError(
        "Position error threshold is too strict, must be larger than "
        "kNumericPrecision.");
  }

  if (params.orientation_error_threshold < kNumericPrecision) {
    return absl::FailedPreconditionError(
        "Orientation error threshold is too strict, must be larger than "
        "kNumericPrecision.");
  }

  if (params.translational_velocity_error_threshold < kNumericPrecision) {
    return absl::FailedPreconditionError(
        "Translational velocity error threshold is too strict, must be "
        "larger "
        "than kNumericPrecision.");
  }

  if (params.angular_velocity_error_threshold < kNumericPrecision) {
    return absl::FailedPreconditionError(
        "Angular velocity error threshold is too strict, must be larger "
        "than kNumericPrecision.");
  }

  if (params.joint_velocity_threshold < kNumericPrecision) {
    return absl::FailedPreconditionError(
        "Joint velocity error threshold is too strict, must be larger "
        "than kNumericPrecision.");
  }

  if (params.translational_distance_along.has_value()) {
    if (params.translational_distance_along->norm() < kMinVectorNorm) {
      return absl::FailedPreconditionError(
          "Translational distance along must have norm larger than "
          "kMinVectorNorm.");
    }

    params.translational_distance_along->normalize();
  }

  if (params.force_direction_1_in_task.norm() < kMinVectorNorm) {
    return absl::FailedPreconditionError(
        "Force direction 1 cannot have a zero norm.");
  }
  params.force_direction_1_in_task.normalize();

  if (params.force_direction_2_in_task.norm() < kMinVectorNorm) {
    return absl::FailedPreconditionError(
        "Force direction 2 cannot have a zero norm.");
  }
  params.force_direction_2_in_task.normalize();

  if (params.force_direction_3_in_task.norm() < kMinVectorNorm) {
    return absl::FailedPreconditionError(
        "Force direction 3 cannot have a zero norm.");
  }
  params.force_direction_3_in_task.normalize();

  if (params.position_error_direction_1_in_task.norm() < kMinVectorNorm) {
    return absl::FailedPreconditionError(
        "Position error direction 1 cannot have a zero norm.");
  }
  params.position_error_direction_1_in_task.normalize();

  if (params.position_error_direction_2_in_task.norm() < kMinVectorNorm) {
    return absl::FailedPreconditionError(
        "Position error direction 2 cannot have a zero norm.");
  }
  params.position_error_direction_2_in_task.normalize();

  if (params.position_error_direction_3_in_task.norm() < kMinVectorNorm) {
    return absl::FailedPreconditionError(
        "Position error direction 3 cannot have a zero norm.");
  }
  params.position_error_direction_3_in_task.normalize();

  if (params.orientation_error_axis_1_in_task.norm() < kMinVectorNorm) {
    return absl::FailedPreconditionError(
        "Orientation error axis 1 cannot have a zero norm.");
  }
  params.orientation_error_axis_1_in_task.normalize();

  if (params.orientation_error_axis_2_in_task.norm() < kMinVectorNorm) {
    return absl::FailedPreconditionError(
        "Orientation error axis 2 cannot have a zero norm.");
  }
  params.orientation_error_axis_2_in_task.normalize();

  if (params.orientation_error_axis_3_in_task.norm() < kMinVectorNorm) {
    return absl::FailedPreconditionError(
        "Orientation error axis 3 cannot have a zero norm.");
  }
  params.orientation_error_axis_3_in_task.normalize();

  return absl::OkStatus();
}

RealtimeStatusOr<RealTimeCartesianTarget> ToRealTimeCartesianTarget(
    const Pose3d& base_t_tip_last_commanded,
    const CartesianTarget& nrt_target) {
  RealTimeCartesianTarget rt_target;

  rt_target.robot_tip_t_robot_tool = nrt_target.robot_tip_t_robot_tool;
  rt_target.robot_base_t_task = nrt_target.robot_base_t_task;

  if (!nrt_target.task_t_tool_reference_pose.has_value()) {
    Pose3d base_t_tool =
        base_t_tip_last_commanded * nrt_target.robot_tip_t_robot_tool;
    rt_target.task_t_tool_reference_orientation =
        nrt_target.robot_base_t_task.quaternion().inverse() *
        base_t_tool.quaternion();
    rt_target.task_t_tool_reference_position =
        nrt_target.robot_base_t_task.inverse() * base_t_tool.translation();
  } else {
    rt_target.task_t_tool_reference_orientation =
        nrt_target.task_t_tool_reference_pose->quaternion();
    rt_target.task_t_tool_reference_position =
        nrt_target.task_t_tool_reference_pose->translation();
  }

  if (!nrt_target.cartesian_damping.has_value()) {
    return FailedPreconditionError("Cartesian damping must be set.");
  }
  rt_target.cartesian_damping = *nrt_target.cartesian_damping;

  rt_target.wrench_selection_matrix = nrt_target.wrench_selection_matrix;

  // Invert the virtual Cartesian inertia matrix
  [&]() INTRINSIC_SUPPRESS_REALTIME_CHECK {
    // TODO(b/394004292): Avoid malloc.
    rt_target.virtual_cartesian_inertia_inverse =
        nrt_target.virtual_cartesian_inertia.inverse();
  }();
  rt_target.cartesian_stiffness = nrt_target.cartesian_stiffness;
  rt_target.tool_reference_twist = nrt_target.tool_reference_twist;
  rt_target.tool_reference_acceleration =
      nrt_target.tool_reference_acceleration;
  rt_target.tool_reference_wrench = nrt_target.tool_reference_wrench;

  return rt_target;
}

RealtimeStatusOr<RealTimeNullspaceTarget> ToRealTimeNullspaceTarget(
    const NullspaceTarget& nrt_target, const JointStatePVA& joint_state) {
  RealTimeNullspaceTarget rt_target;

  INTRINSIC_RT_RETURN_IF_ERROR(
      rt_target.joint_state.SetSize(joint_state.size()));
  rt_target.joint_state.velocity.setZero();
  rt_target.joint_state.acceleration.setZero();

  if (nrt_target.joint_position_nullspace.has_value()) {
    if (nrt_target.joint_position_nullspace->size() != joint_state.size()) {
      return FailedPreconditionError(
          "Dimension of nullspace reference must be equal to the number of "
          "joints.");
    }
    rt_target.joint_state.position =
        nrt_target.joint_position_nullspace.value();
  } else {
    rt_target.joint_state.position = joint_state.position;
  }

  if (!nrt_target.nullspace_stiffness.has_value()) {
    return FailedPreconditionError(
        "Nullspace stiffness must be provided in "
        "ToRealTimeNullspaceTarget().");
  }
  rt_target.nullspace_stiffness = *nrt_target.nullspace_stiffness;

  if (!nrt_target.nullspace_damping.has_value()) {
    return FailedPreconditionError(
        "Nullspace damping must be provided in ToRealTimeNullspaceTarget().");
  }
  rt_target.nullspace_damping = *nrt_target.nullspace_damping;

  return rt_target;
}

RealtimeStatusOr<RealTimeConstraints> ToRealTimeConstraints(
    const Constraints& nrt_constraints, const JointLimits& system_limits,
    const CartesianLimits& default_cartesian_limits, int njoints) {
  RealTimeConstraints rt_constraints;

  // Assign user-provided Cartesian limits without a check, since there are no
  // "Cartesian system limits".
  rt_constraints.cart_limits =
      nrt_constraints.cartesian_limits.value_or(default_cartesian_limits);

  if (nrt_constraints.joint_limits.has_value()) {
    if (nrt_constraints.joint_limits->size() != njoints) {
      return FailedPreconditionError(
          "Provided limits have wrong number of joints.");
    }
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const LimitCheckResult within_limits,
        IsWithinLimits(nrt_constraints.joint_limits.value(), system_limits));
    if (!within_limits) {
      return InvalidArgumentError(
          "Provided joint_limits violate maximum limits.");
    }
    rt_constraints.joint_limits = *nrt_constraints.joint_limits;
  } else {
    rt_constraints.joint_limits = system_limits;
  }

  // Eliminate "infinity" from the limits to avoid numerical issues.
  const double kHighestDouble = 1e20;
  const double kLowestDouble = -1e20;
  rt_constraints.joint_limits.max_position.array() =
      rt_constraints.joint_limits.max_position.array().min(kHighestDouble);
  rt_constraints.joint_limits.min_position.array() =
      rt_constraints.joint_limits.min_position.array().max(kLowestDouble);
  rt_constraints.joint_limits.max_velocity.array() =
      rt_constraints.joint_limits.max_velocity.array().min(kHighestDouble);
  rt_constraints.joint_limits.max_acceleration.array() =
      rt_constraints.joint_limits.max_acceleration.array().min(kHighestDouble);
  rt_constraints.joint_limits.max_jerk.array() =
      rt_constraints.joint_limits.max_jerk.array().min(kHighestDouble);
  rt_constraints.joint_limits.max_torque.array() =
      rt_constraints.joint_limits.max_torque.array().min(kHighestDouble);

  rt_constraints.cart_limits.min_translational_position.array() =
      rt_constraints.cart_limits.min_translational_position.array().max(
          kLowestDouble);
  rt_constraints.cart_limits.max_translational_position.array() =
      rt_constraints.cart_limits.max_translational_position.array().min(
          kHighestDouble);

  rt_constraints.cart_limits.min_translational_velocity.array() =
      rt_constraints.cart_limits.min_translational_velocity.array().max(
          kLowestDouble);
  rt_constraints.cart_limits.max_translational_velocity.array() =
      rt_constraints.cart_limits.max_translational_velocity.array().min(
          kHighestDouble);
  rt_constraints.cart_limits.max_rotational_velocity = std::min(
      rt_constraints.cart_limits.max_rotational_velocity, kHighestDouble);

  rt_constraints.cart_limits.min_translational_acceleration.array() =
      rt_constraints.cart_limits.min_translational_acceleration.array().max(
          kLowestDouble);
  rt_constraints.cart_limits.max_translational_acceleration.array() =
      rt_constraints.cart_limits.max_translational_acceleration.array().min(
          kHighestDouble);
  rt_constraints.cart_limits.max_rotational_acceleration = std::min(
      rt_constraints.cart_limits.max_rotational_acceleration, kHighestDouble);

  rt_constraints.cart_limits.min_translational_jerk.array() =
      rt_constraints.cart_limits.min_translational_jerk.array().max(
          kLowestDouble);
  rt_constraints.cart_limits.max_translational_jerk.array() =
      rt_constraints.cart_limits.max_translational_jerk.array().min(
          kHighestDouble);
  rt_constraints.cart_limits.max_rotational_jerk =
      std::min(rt_constraints.cart_limits.max_rotational_jerk, kHighestDouble);

  rt_constraints.enforce_cartesian_and_joint_limits =
      nrt_constraints.enforce_cartesian_and_joint_limits;

  return rt_constraints;
}

double ComputeTranslationalDistanceTraveled(
    const Pose3d& ref_t_start, const Pose3d& ref_t_end,
    const std::optional<eigenmath::Vector3d>& along_normalized) {
  const Pose3d start_t_end = ref_t_start.inverse() * ref_t_end;
  const eigenmath::Vector3d& start_p_end = start_t_end.translation();

  if (!along_normalized.has_value()) {
    return start_p_end.norm();
  }
  return start_p_end.dot(*along_normalized);
}

void CopyDoublesToVector3(absl::Span<const double> doubles,
                          intrinsic_proto::Vector3* vector) {
  *vector = ToVectorProto(
      Eigen::Map<const eigenmath::Vector3d>(doubles.data(), doubles.size()));
}

absl::Status CopyDoublesToPose(absl::Span<const double> doubles,
                               intrinsic_proto::Pose* pose) {
  if (doubles.size() != 7) {
    return absl::InvalidArgumentError(
        absl::StrCat("Unsupported number of doubles in CopyDoublesToPose: ",
                     doubles.size()));
    ;
  }

  eigenmath::Quaterniond quat{doubles[3], doubles[4], doubles[5], doubles[6]};
  quat.normalize();
  *pose = intrinsic::ToProto(
      Pose3d(quat, eigenmath::Vector3d(doubles[0], doubles[1], doubles[2])));
  return absl::OkStatus();
}

absl::Status CopyDoublesToTwist(absl::Span<const double> doubles,
                                intrinsic_proto::Twist* twist) {
  if (doubles.size() != 6) {
    return absl::InvalidArgumentError(
        absl::StrCat("Unsupported number of doubles in CopyDoublesToTwist: ",
                     doubles.size()));
  }
  *twist = intrinsic::ToProto(Twist(doubles[0], doubles[1], doubles[2],
                                    doubles[3], doubles[4], doubles[5]));
  return absl::OkStatus();
}

absl::Status CopyDoublesToWrench(absl::Span<const double> doubles,
                                 intrinsic_proto::icon::Wrench* wrench) {
  if (doubles.size() != 6) {
    return absl::InvalidArgumentError(
        absl::StrCat("Unsupported number of doubles in CopyDoublesToWrench: ",
                     doubles.size()));
  }
  *wrench = icon::ToProto(Wrench(doubles[0], doubles[1], doubles[2], doubles[3],
                                 doubles[4], doubles[5]));
  return absl::OkStatus();
}

void CopyPoseToDoubles(const Pose3d& pose, double* doubles) {
  eigenmath::Vector3d translation = pose.translation();
  eigenmath::Quaterniond quaternion = pose.quaternion();
  quaternion.normalize();
  doubles[0] = translation.x();
  doubles[1] = translation.y();
  doubles[2] = translation.z();
  doubles[3] = quaternion.w();
  doubles[4] = quaternion.x();
  doubles[5] = quaternion.y();
  doubles[6] = quaternion.z();
}

absl::StatusOr<intrinsic_proto::icon::actions::proto::CartesianAdmittanceStatus>
ToStreamingOutputStatusProto(
    const AdmittanceActionStreamingOutputStatus& status) {
  CartesianAdmittanceInfo::StreamingOutput proto;

  CopyDoublesToVector3(status.error_state_position,
                       proto.mutable_error_state_position());
  CopyDoublesToVector3(status.error_state_orientation,
                       proto.mutable_error_state_orientation());
  CopyDoublesToVector3(status.error_state_translational_velocity,
                       proto.mutable_error_state_translational_velocity());
  CopyDoublesToVector3(status.error_state_angular_velocity,
                       proto.mutable_error_state_angular_velocity());
  CopyDoublesToVector3(status.error_state_force,
                       proto.mutable_error_state_force());
  CopyDoublesToVector3(status.error_state_torque,
                       proto.mutable_error_state_torque());
  CopyDoublesToVector3(
      status.previous_cartesian_translational_acceleration_command,
      proto.mutable_previous_cartesian_translational_acceleration_command());
  CopyDoublesToVector3(
      status.previous_cartesian_angular_acceleration_command,
      proto.mutable_previous_cartesian_angular_acceleration_command());
  CopyDoublesToVector3(status.sensed_force_at_tool_in_task_frame,
                       proto.mutable_sensed_force_at_tool_in_task_frame());
  CopyDoublesToVector3(status.sensed_torque_at_tool_in_task_frame,
                       proto.mutable_sensed_torque_at_tool_in_task_frame());
  CopyDoublesToVector3(
      status.post_sensor_dynamics_force_in_task_frame,
      proto.mutable_post_sensor_dynamics_force_in_task_frame());
  CopyDoublesToVector3(
      status.post_sensor_dynamics_torque_in_task_frame,
      proto.mutable_post_sensor_dynamics_torque_in_task_frame());
  proto.set_settled_for_seconds(status.settled_for_seconds);
  proto.set_sensed_force_magnitude(status.sensed_force_magnitude);
  proto.set_sensed_torque_magnitude(status.sensed_torque_magnitude);
  proto.set_translational_distance_traveled(
      status.translational_distance_traveled);
  proto.set_maximum_translational_displacement_in_time_window(
      status.maximum_translational_displacement_in_time_window);
  proto.set_is_settled(status.is_settled);

  INTR_RETURN_IF_ERROR(CopyDoublesToPose(
      status.robot_tip_t_robot_tool, proto.mutable_robot_tip_t_robot_tool()));
  INTR_RETURN_IF_ERROR(CopyDoublesToPose(status.robot_base_t_task,
                                         proto.mutable_robot_base_t_task()));
  INTR_RETURN_IF_ERROR(
      CopyDoublesToPose(status.task_t_tool_reference_pose,
                        proto.mutable_task_t_tool_reference_pose()));
  INTR_RETURN_IF_ERROR(CopyDoublesToTwist(
      status.tool_reference_twist, proto.mutable_tool_reference_twist()));
  INTR_RETURN_IF_ERROR(CopyDoublesToWrench(
      status.tool_reference_wrench, proto.mutable_tool_reference_wrench()));

  return proto;
}

eigenmath::Matrix6d TransformTaskMatrix(const Pose3d& a_t_b,
                                        const eigenmath::Matrix6d& matrix_b) {
  eigenmath::Matrix6d big_R = eigenmath::Matrix6d::Identity();
  big_R.topLeftCorner(3, 3) = a_t_b.rotationMatrix();
  big_R.bottomRightCorner(3, 3) = a_t_b.rotationMatrix();
  return big_R * matrix_b * big_R.transpose();
}

eigenmath::Vector6d RotateVector(const eigenmath::Quaterniond& a_R_b,
                                 const eigenmath::Vector6d& b_vec) {
  eigenmath::Vector6d a_vec;
  a_vec.head<3>() = a_R_b.matrix() * b_vec.head<3>();
  a_vec.tail<3>() = a_R_b.matrix() * b_vec.tail<3>();
  return a_vec;
}

CartesianErrorState RotateCartesianErrorState(
    const eigenmath::Quaterniond& a_r_b,
    const CartesianErrorState& error_state_in_b) {
  CartesianErrorState error_state_in_a;

  error_state_in_a.pose = RotateVector(a_r_b, error_state_in_b.pose);
  error_state_in_a.twist = RotateVector(a_r_b, error_state_in_b.twist);
  error_state_in_a.wrench = RotateVector(a_r_b, error_state_in_b.wrench);
  return error_state_in_a;
}

}  // namespace cartesian_impedance

}  // namespace intrinsic::icon
