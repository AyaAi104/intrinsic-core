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

#include "intrinsic/icon/actions/cartesian_impedance_parameters.h"

#include <optional>
#include <variant>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/cartesian_impedance.pb.h"
#include "intrinsic/icon/actions/cartesian_impedance_reference_generators.pb.h"
#include "intrinsic/icon/proto/cart_space.pb.h"
#include "intrinsic/icon/proto/cart_space_conversion.h"
#include "intrinsic/icon/proto/eigen_conversion.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/icon/proto/matrix.pb.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/math/twist.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

namespace {

constexpr double kMinVectorNorm = 1e-5;

// Visitor dedicated to parsing Cartesian Impedance reference generator
// parameters into protos.
struct SetCartesianReferenceGeneratorConfig {
  intrinsic_proto::icon::actions::proto::ReferenceGenerators::Cartesian& proto;
  void operator()(const cartesian_impedance::
                      CartesianPositionReflexxesImpedanceGeneratorParameters&
                          params) const {
    *proto.mutable_position_reflexxes() = ToProto(params);
  }

  void operator()(const cartesian_impedance::
                      CartesianVelocityReflexxesImpedanceGeneratorParameters&
                          params) const {
    *proto.mutable_velocity_reflexxes() = ToProto(params);
  }

  void operator()(const cartesian_impedance::SimpleImpedanceGeneratorParameters&
                      params) const {
    *proto.mutable_simple_lowpass() = ToProto(params);
  }

  void operator()(
      const cartesian_impedance::QuinticSplineCartesianGeneratorParameters&
          params) const {
    *proto.mutable_quintic_spline() = ToProto(params);
  }
};

// Visitor dedicated to parsing nullspace reference generator parameters into
// protos.
struct SetNullspaceReferenceGeneratorConfig {
  intrinsic_proto::icon::actions::proto::ReferenceGenerators::Nullspace& proto;

  void operator()(
      const cartesian_impedance::ReflexxesNullspaceReferenceGeneratorParameters&
          params) const {
    *proto.mutable_reflexxes() = ToProto(params);
  }

  void operator()(
      const cartesian_impedance::SimpleNullspaceReferenceGeneratorParameters&
          params) const {
    *proto.mutable_simple_lowpass() = ToProto(params);
  }

  void operator()(
      const cartesian_impedance::
          QuinticSplineNullspaceReferenceGeneratorParameters& params) const {
    *proto.mutable_quintic_spline() = ToProto(params);
  }
};

}  // namespace

intrinsic_proto::icon::actions::proto::QuinticSplineCartesianReferenceGenerator
ToProto(const cartesian_impedance::QuinticSplineCartesianGeneratorParameters&
            params) {
  intrinsic_proto::icon::actions::proto::
      QuinticSplineCartesianReferenceGenerator proto;
  proto.set_movement_duration_seconds(params.movement_duration_seconds);
  return proto;
}

cartesian_impedance::QuinticSplineCartesianGeneratorParameters FromProto(
    const intrinsic_proto::icon::actions::proto::
        QuinticSplineCartesianReferenceGenerator& proto) {
  return {.movement_duration_seconds = proto.movement_duration_seconds()};
}

intrinsic_proto::icon::actions::proto::SimpleImpedanceReferenceGenerator
ToProto(const cartesian_impedance::SimpleImpedanceGeneratorParameters& params) {
  intrinsic_proto::icon::actions::proto::SimpleImpedanceReferenceGenerator
      proto;
  proto.set_lowpass_filter_constant(params.lowpass_filter_constant);
  return proto;
}

cartesian_impedance::SimpleImpedanceGeneratorParameters FromProto(
    const intrinsic_proto::icon::actions::proto::
        SimpleImpedanceReferenceGenerator& proto) {
  if (!proto.has_lowpass_filter_constant()) {
    return {};  // Uses default values for lowpass filter.
  }
  return {.lowpass_filter_constant = proto.lowpass_filter_constant()};
}

intrinsic_proto::icon::actions::proto::QuinticSplineNullspaceReferenceGenerator
ToProto(const cartesian_impedance::
            QuinticSplineNullspaceReferenceGeneratorParameters& params) {
  intrinsic_proto::icon::actions::proto::
      QuinticSplineNullspaceReferenceGenerator proto;
  proto.set_movement_duration_seconds(params.movement_duration_seconds);
  return proto;
}

cartesian_impedance::QuinticSplineNullspaceReferenceGeneratorParameters
FromProto(const intrinsic_proto::icon::actions::proto::
              QuinticSplineNullspaceReferenceGenerator& proto) {
  return {.movement_duration_seconds = proto.movement_duration_seconds()};
}

intrinsic_proto::icon::actions::proto::SimpleNullspaceReferenceGenerator
ToProto(const cartesian_impedance::SimpleNullspaceReferenceGeneratorParameters&
            params) {
  intrinsic_proto::icon::actions::proto::SimpleNullspaceReferenceGenerator
      proto;
  proto.set_lowpass_filter_constant(params.lowpass_filter_constant);
  return proto;
}

cartesian_impedance::SimpleNullspaceReferenceGeneratorParameters FromProto(
    const intrinsic_proto::icon::actions::proto::
        SimpleNullspaceReferenceGenerator& proto) {
  if (!proto.has_lowpass_filter_constant()) {
    return {};  // Uses default value for lowpass filter.
  }
  return {.lowpass_filter_constant = proto.lowpass_filter_constant()};
}

intrinsic_proto::icon::actions::proto::
    CartesianPositionReflexxesImpedanceReferenceGenerator
    ToProto(
        const cartesian_impedance::
            CartesianPositionReflexxesImpedanceGeneratorParameters& params) {
  return {};  // TODO(b/190081664): add parameters.
}

cartesian_impedance::CartesianPositionReflexxesImpedanceGeneratorParameters
FromProto(const intrinsic_proto::icon::actions::proto::
              CartesianPositionReflexxesImpedanceReferenceGenerator& proto) {
  return {};  // TODO(b/190081664): add parameters.
}

intrinsic_proto::icon::actions::proto::
    CartesianVelocityReflexxesImpedanceReferenceGenerator
    ToProto(
        const cartesian_impedance::
            CartesianVelocityReflexxesImpedanceGeneratorParameters& params) {
  return {};  // TODO(b/190081664): add parameters.
}

cartesian_impedance::CartesianVelocityReflexxesImpedanceGeneratorParameters
FromProto(const intrinsic_proto::icon::actions::proto::
              CartesianVelocityReflexxesImpedanceReferenceGenerator& proto) {
  return {};  // TODO(b/190081664): add parameters.
}

intrinsic_proto::icon::actions::proto::ReflexxesNullspaceReferenceGenerator
ToProto(
    const cartesian_impedance::ReflexxesNullspaceReferenceGeneratorParameters&
        params) {
  return {};  // TODO(b/190081664): add parameters.
}

cartesian_impedance::ReflexxesNullspaceReferenceGeneratorParameters FromProto(
    const intrinsic_proto::icon::actions::proto::
        ReflexxesNullspaceReferenceGenerator& proto) {
  return {};  // TODO(b/190081664): add parameters.
}

intrinsic_proto::icon::actions::proto::CartesianTarget ToProto(
    const cartesian_impedance::CartesianTarget& params) {
  intrinsic_proto::icon::actions::proto::CartesianTarget proto_params;
  *proto_params.mutable_robot_tip_t_robot_tool() =
      intrinsic::icon::ToProto(params.robot_tip_t_robot_tool);

  *proto_params.mutable_robot_base_t_task() =
      intrinsic::icon::ToProto(params.robot_base_t_task);

  if (params.task_t_tool_reference_pose.has_value()) {
    *proto_params.mutable_task_t_tool_reference_pose() =
        intrinsic::icon::ToProto(params.task_t_tool_reference_pose.value());
  }

  *proto_params.mutable_tool_reference_twist() =
      intrinsic::icon::ToProto(params.tool_reference_twist);

  *proto_params.mutable_tool_reference_acceleration() =
      intrinsic::icon::ToProto(params.tool_reference_acceleration);

  *proto_params.mutable_tool_reference_wrench() =
      intrinsic::icon::ToProto(params.tool_reference_wrench);

  *proto_params.mutable_cartesian_stiffness() =
      intrinsic::icon::ToProto(params.cartesian_stiffness);

  *proto_params.mutable_virtual_cartesian_inertia() =
      ToProto(params.virtual_cartesian_inertia);

  if (params.cartesian_damping.has_value()) {
    *proto_params.mutable_cartesian_damping() =
        ToProto(params.cartesian_damping.value());
  }

  *proto_params.mutable_wrench_selection_matrix() =
      ToProto(params.wrench_selection_matrix);

  return proto_params;
}

intrinsic_proto::icon::actions::proto::NullspaceTarget ToProto(
    const cartesian_impedance::NullspaceTarget& params) {
  intrinsic_proto::icon::actions::proto::NullspaceTarget proto_params;

  if (params.nullspace_stiffness.has_value()) {
    proto_params.set_nullspace_stiffness(params.nullspace_stiffness.value());
  }

  if (params.nullspace_damping.has_value()) {
    proto_params.set_nullspace_damping(params.nullspace_damping.value());
  }

  if (params.joint_position_nullspace.has_value()) {
    *proto_params.mutable_nullspace_reference() =
        ToJointVecProto(params.joint_position_nullspace.value());
  }
  return proto_params;
}

intrinsic_proto::icon::actions::proto::AlgorithmConfiguration ToProto(
    const cartesian_impedance::AlgorithmConfiguration& params) {
  intrinsic_proto::icon::actions::proto::AlgorithmConfiguration proto_params;

  proto_params.set_jacobian_pinv_damping(params.jacobian_pinv_damping);

  *proto_params.mutable_sensed_wrench_deadband() =
      ToProto(params.sensed_wrench_deadband);

  *proto_params.mutable_pose_error_integrator_gain() =
      ToProto(params.pose_error_integrator_gain);

  *proto_params.mutable_pose_error_integrator_bound() =
      ToProto(params.pose_error_integrator_bound);

  proto_params.set_post_sensor_dynamic_load_multiplier(
      params.post_sensor_dynamic_load_multiplier);

  return proto_params;
}

intrinsic_proto::icon::actions::proto::Constraints ToProto(
    const cartesian_impedance::Constraints& params) {
  intrinsic_proto::icon::actions::proto::Constraints proto_params;

  if (params.joint_limits.has_value()) {
    *proto_params.mutable_joint_limits() = ToProto(params.joint_limits.value());
  }

  if (params.cartesian_limits.has_value()) {
    *proto_params.mutable_cartesian_limits() =
        ToProto(params.cartesian_limits.value());
  }

  proto_params.set_enforce_cartesian_and_joint_limits(
      params.enforce_cartesian_and_joint_limits);

  return proto_params;
}

intrinsic_proto::icon::actions::proto::StateVariableConfiguration ToProto(
    const cartesian_impedance::StateVariableConfiguration& params) {
  intrinsic_proto::icon::actions::proto::StateVariableConfiguration
      proto_params;
  proto_params.set_joint_velocity_threshold(params.joint_velocity_threshold);

  proto_params.set_position_error_threshold(params.position_error_threshold);

  proto_params.set_orientation_error_threshold(
      params.orientation_error_threshold);

  proto_params.set_translation_velocity_error_threshold(
      params.translational_velocity_error_threshold);

  proto_params.set_angular_velocity_error_threshold(
      params.angular_velocity_error_threshold);

  if (params.translational_distance_along.has_value()) {
    *proto_params.mutable_translational_distance_along() =
        ToVectorProto(*params.translational_distance_along);
  }

  proto_params.set_time_window_for_maximum_displacement_sec(
      params.time_window_for_maximum_displacement);

  proto_params.set_force_settled_band(params.force_settled_band);

  *proto_params.mutable_force_direction_1_in_task() =
      ToVectorProto(params.force_direction_1_in_task);
  *proto_params.mutable_force_direction_2_in_task() =
      ToVectorProto(params.force_direction_2_in_task);
  *proto_params.mutable_force_direction_3_in_task() =
      ToVectorProto(params.force_direction_3_in_task);
  *proto_params.mutable_torque_axis_1_in_task() =
      ToVectorProto(params.torque_axis_1_in_task);
  *proto_params.mutable_torque_axis_2_in_task() =
      ToVectorProto(params.torque_axis_2_in_task);
  *proto_params.mutable_torque_axis_3_in_task() =
      ToVectorProto(params.torque_axis_3_in_task);
  *proto_params.mutable_position_error_direction_1_in_task() =
      ToVectorProto(params.position_error_direction_1_in_task);
  *proto_params.mutable_position_error_direction_2_in_task() =
      ToVectorProto(params.position_error_direction_2_in_task);
  *proto_params.mutable_position_error_direction_3_in_task() =
      ToVectorProto(params.position_error_direction_3_in_task);
  *proto_params.mutable_orientation_error_axis_1_in_task() =
      ToVectorProto(params.orientation_error_axis_1_in_task);
  *proto_params.mutable_orientation_error_axis_2_in_task() =
      ToVectorProto(params.orientation_error_axis_2_in_task);
  *proto_params.mutable_orientation_error_axis_3_in_task() =
      ToVectorProto(params.orientation_error_axis_3_in_task);

  return proto_params;
}

intrinsic_proto::icon::actions::proto::ReferenceGenerators ToProto(
    const cartesian_impedance::ReferenceGenerators& params) {
  intrinsic_proto::icon::actions::proto::ReferenceGenerators proto_params;
  std::visit(
      SetCartesianReferenceGeneratorConfig{
          .proto = *proto_params.mutable_cartesian()},
      params.cartesian_reference_generator);

  std::visit(
      SetNullspaceReferenceGeneratorConfig{
          .proto = *proto_params.mutable_nullspace()},
      params.nullspace_reference_generator);

  return proto_params;
}

intrinsic_proto::icon::actions::proto::CartesianImpedanceParameters ToProto(
    const CartesianImpedanceParameters& params) {
  intrinsic_proto::icon::actions::proto::CartesianImpedanceParameters
      proto_params;

  *proto_params.mutable_cartesian_target() = ToProto(params.cartesian_target);
  *proto_params.mutable_nullspace_target() = ToProto(params.nullspace_target);
  *proto_params.mutable_algorithm_configuration() =
      ToProto(params.algorithm_configuration);
  *proto_params.mutable_constraints() = ToProto(params.constraints);
  *proto_params.mutable_state_variable_configuration() =
      ToProto(params.state_variable_configuration);
  *proto_params.mutable_reference_generators() =
      ToProto(params.reference_generators);

  return proto_params;
}

absl::StatusOr<cartesian_impedance::CartesianTarget> FromProto(
    const intrinsic_proto::icon::actions::proto::CartesianTarget&
        proto_params) {
  INTRINSIC_ASSERT_NON_REALTIME();

  cartesian_impedance::CartesianTarget params;
  INTR_ASSIGN_OR_RETURN(params.robot_tip_t_robot_tool,
                        FromProto(proto_params.robot_tip_t_robot_tool()));

  if (proto_params.has_robot_base_t_task()) {
    INTR_ASSIGN_OR_RETURN(params.robot_base_t_task,
                          FromProto(proto_params.robot_base_t_task()));
  }

  if (proto_params.has_task_t_tool_reference_pose()) {
    INTR_ASSIGN_OR_RETURN(params.task_t_tool_reference_pose,
                          FromProto(proto_params.task_t_tool_reference_pose()));
  }

  params.tool_reference_twist = FromProto(proto_params.tool_reference_twist());

  params.tool_reference_acceleration =
      FromProto(proto_params.tool_reference_acceleration());

  params.tool_reference_wrench =
      FromProto(proto_params.tool_reference_wrench());

  // Check for existence and copy cartesian stiffness.
  if (!proto_params.has_cartesian_stiffness()) {
    return absl::InvalidArgumentError(
        "CartesianImpedanceParameters must contain the cartesian_stiffness.");
  }
  INTR_ASSIGN_OR_RETURN(params.cartesian_stiffness,
                        FromProto(proto_params.cartesian_stiffness()),
                        _ << "Copying a 6x6 stiffness matrix failed.");

  // Check for existence and copy cartesian inertia inverse.
  if (!proto_params.has_virtual_cartesian_inertia()) {
    return absl::InvalidArgumentError(
        "CartesianImpedanceParameters must contain the "
        "  virtual_cartesian_inertia.");
  }
  INTR_ASSIGN_OR_RETURN(params.virtual_cartesian_inertia,
                        FromProto(proto_params.virtual_cartesian_inertia()),
                        _ << "Copying a 6x6 inertia matrix inverse failed.");

  // Check for existence and copy cartesian damping.
  if (proto_params.has_cartesian_damping()) {
    INTR_ASSIGN_OR_RETURN(params.cartesian_damping,
                          FromProto(proto_params.cartesian_damping()),
                          _ << "Copying a 6x6 damping matrix failed.");
  }

  // Check for existence and copy wrench selection matrix.
  if (proto_params.has_wrench_selection_matrix()) {
    INTR_ASSIGN_OR_RETURN(params.wrench_selection_matrix,
                          FromProto(proto_params.wrench_selection_matrix()),
                          _ << "Copying a 6x6 wrench selection matrix failed.");
  }

  return params;
}

absl::StatusOr<cartesian_impedance::NullspaceTarget> FromProto(
    const intrinsic_proto::icon::actions::proto::NullspaceTarget&
        proto_params) {
  INTRINSIC_ASSERT_NON_REALTIME();
  cartesian_impedance::NullspaceTarget params;

  if (proto_params.has_nullspace_stiffness()) {
    params.nullspace_stiffness = proto_params.nullspace_stiffness();
  }
  if (proto_params.has_nullspace_damping()) {
    params.nullspace_damping = proto_params.nullspace_damping();
  }

  if (proto_params.has_nullspace_reference()) {
    INTR_ASSIGN_OR_RETURN(eigenmath::VectorNd temp,
                          FromProto(proto_params.nullspace_reference()));
    params.joint_position_nullspace = temp;
  }

  return params;
}

absl::StatusOr<cartesian_impedance::AlgorithmConfiguration> FromProto(
    const intrinsic_proto::icon::actions::proto::AlgorithmConfiguration&
        proto_params) {
  INTRINSIC_ASSERT_NON_REALTIME();
  cartesian_impedance::AlgorithmConfiguration params;

  if (proto_params.has_jacobian_pinv_damping()) {
    params.jacobian_pinv_damping = proto_params.jacobian_pinv_damping();
  }

  if (proto_params.has_sensed_wrench_deadband()) {
    params.sensed_wrench_deadband =
        FromProto(proto_params.sensed_wrench_deadband());
  }

  if (proto_params.has_pose_error_integrator_gain()) {
    params.pose_error_integrator_gain =
        FromProto(proto_params.pose_error_integrator_gain());
  }

  if (proto_params.has_pose_error_integrator_bound()) {
    params.pose_error_integrator_bound =
        FromProto(proto_params.pose_error_integrator_bound());
  }

  if (proto_params.has_post_sensor_dynamic_load_multiplier()) {
    params.post_sensor_dynamic_load_multiplier =
        proto_params.post_sensor_dynamic_load_multiplier();
  }

  return params;
}

absl::StatusOr<cartesian_impedance::Constraints> FromProto(
    const intrinsic_proto::icon::actions::proto::Constraints& proto_params) {
  INTRINSIC_ASSERT_NON_REALTIME();

  cartesian_impedance::Constraints params;
  if (proto_params.has_joint_limits()) {
    INTR_ASSIGN_OR_RETURN(params.joint_limits,
                          intrinsic::FromProto(proto_params.joint_limits()));
  }

  if (proto_params.has_cartesian_limits()) {
    INTR_ASSIGN_OR_RETURN(params.cartesian_limits,
                          FromProto(proto_params.cartesian_limits()));
  }

  if (proto_params.has_enforce_cartesian_and_joint_limits()) {
    params.enforce_cartesian_and_joint_limits =
        proto_params.enforce_cartesian_and_joint_limits();
  }

  return params;
}

absl::StatusOr<cartesian_impedance::StateVariableConfiguration> FromProto(
    const intrinsic_proto::icon::actions::proto::StateVariableConfiguration&
        proto_params) {
  INTRINSIC_ASSERT_NON_REALTIME();
  cartesian_impedance::StateVariableConfiguration params;

  if (proto_params.has_position_error_threshold()) {
    params.position_error_threshold = proto_params.position_error_threshold();
  }

  if (proto_params.has_orientation_error_threshold()) {
    params.orientation_error_threshold =
        proto_params.orientation_error_threshold();
  }

  if (proto_params.has_translation_velocity_error_threshold()) {
    params.translational_velocity_error_threshold =
        proto_params.translation_velocity_error_threshold();
  }

  if (proto_params.has_angular_velocity_error_threshold()) {
    params.angular_velocity_error_threshold =
        proto_params.angular_velocity_error_threshold();
  }

  if (proto_params.has_joint_velocity_threshold()) {
    params.joint_velocity_threshold = proto_params.joint_velocity_threshold();
  }

  if (proto_params.has_translational_distance_along()) {
    eigenmath::Vector3d vector =
        FromProto(proto_params.translational_distance_along());
    if (vector.norm() < kMinVectorNorm) {
      return absl::FailedPreconditionError(
          absl::StrCat("The `translational_distance_along` must have a norm "
                       "greater than ",
                       kMinVectorNorm));
    }
    vector.normalize();
    params.translational_distance_along = vector;
  }

  if (proto_params.has_time_window_for_maximum_displacement_sec()) {
    params.time_window_for_maximum_displacement =
        proto_params.time_window_for_maximum_displacement_sec();
  }

  if (proto_params.has_force_settled_band()) {
    params.force_settled_band = proto_params.force_settled_band();
  }

  if (proto_params.has_force_direction_1_in_task()) {
    params.force_direction_1_in_task =
        FromProto(proto_params.force_direction_1_in_task());
  }
  if (proto_params.has_force_direction_2_in_task()) {
    params.force_direction_2_in_task =
        FromProto(proto_params.force_direction_2_in_task());
  }
  if (proto_params.has_force_direction_3_in_task()) {
    params.force_direction_3_in_task =
        FromProto(proto_params.force_direction_3_in_task());
  }
  if (proto_params.has_torque_axis_1_in_task()) {
    params.torque_axis_1_in_task =
        FromProto(proto_params.torque_axis_1_in_task());
  }
  if (proto_params.has_torque_axis_2_in_task()) {
    params.torque_axis_2_in_task =
        FromProto(proto_params.torque_axis_2_in_task());
  }
  if (proto_params.has_torque_axis_3_in_task()) {
    params.torque_axis_3_in_task =
        FromProto(proto_params.torque_axis_3_in_task());
  }
  if (proto_params.has_position_error_direction_1_in_task()) {
    params.position_error_direction_1_in_task =
        FromProto(proto_params.position_error_direction_1_in_task());
  }
  if (proto_params.has_position_error_direction_2_in_task()) {
    params.position_error_direction_2_in_task =
        FromProto(proto_params.position_error_direction_2_in_task());
  }
  if (proto_params.has_position_error_direction_3_in_task()) {
    params.position_error_direction_3_in_task =
        FromProto(proto_params.position_error_direction_3_in_task());
  }
  if (proto_params.has_orientation_error_axis_1_in_task()) {
    params.orientation_error_axis_1_in_task =
        FromProto(proto_params.orientation_error_axis_1_in_task());
  }
  if (proto_params.has_orientation_error_axis_2_in_task()) {
    params.orientation_error_axis_2_in_task =
        FromProto(proto_params.orientation_error_axis_2_in_task());
  }
  if (proto_params.has_orientation_error_axis_3_in_task()) {
    params.orientation_error_axis_3_in_task =
        FromProto(proto_params.orientation_error_axis_3_in_task());
  }

  return params;
}

absl::StatusOr<cartesian_impedance::ReferenceGenerators> FromProto(
    const intrinsic_proto::icon::actions::proto::ReferenceGenerators&
        proto_params) {
  INTRINSIC_ASSERT_NON_REALTIME();
  cartesian_impedance::ReferenceGenerators params;
  switch (proto_params.cartesian().reference_generator_case()) {
    case intrinsic_proto::icon::actions::proto::ReferenceGenerators::Cartesian::
        kPositionReflexxes: {
      params.cartesian_reference_generator =
          FromProto(proto_params.cartesian().position_reflexxes());
      break;
    }
    case intrinsic_proto::icon::actions::proto::ReferenceGenerators::Cartesian::
        kVelocityReflexxes: {
      params.cartesian_reference_generator =
          FromProto(proto_params.cartesian().velocity_reflexxes());
      break;
    }
    case intrinsic_proto::icon::actions::proto::ReferenceGenerators::Cartesian::
        kQuinticSpline: {
      params.cartesian_reference_generator =
          FromProto(proto_params.cartesian().quintic_spline());
      break;
    }
    case intrinsic_proto::icon::actions::proto::ReferenceGenerators::Cartesian::
        kSimpleLowpass: {
      params.cartesian_reference_generator =
          FromProto(proto_params.cartesian().simple_lowpass());
      break;
    }
    case intrinsic_proto::icon::actions::proto::ReferenceGenerators::Cartesian::
        REFERENCE_GENERATOR_NOT_SET: {
      // This case should not happen in g3, possibly only in case of a corrupted
      // proto.
      return absl::FailedPreconditionError(
          "Fatal, reached REFERENCE_GENERATOR_NOT_SET.");
      break;
    }
  }

  switch (proto_params.nullspace().reference_generator_case()) {
    case intrinsic_proto::icon::actions::proto::ReferenceGenerators::Nullspace::
        kReflexxes: {
      params.nullspace_reference_generator =
          FromProto(proto_params.nullspace().reflexxes());
      break;
    }
    case intrinsic_proto::icon::actions::proto::ReferenceGenerators::Nullspace::
        kQuinticSpline: {
      params.nullspace_reference_generator =
          params.nullspace_reference_generator =
              FromProto(proto_params.nullspace().quintic_spline());
      break;
    }
    case intrinsic_proto::icon::actions::proto::ReferenceGenerators::Nullspace::
        kSimpleLowpass: {
      params.nullspace_reference_generator =
          FromProto(proto_params.nullspace().simple_lowpass());
      break;
    }
    case intrinsic_proto::icon::actions::proto::ReferenceGenerators::Nullspace::
        REFERENCE_GENERATOR_NOT_SET: {
      // This case should not happen in g3, possibly only in case of a corrupted
      // proto.
      return absl::FailedPreconditionError(
          "Fatal, reached REFERENCE_GENERATOR_NOT_SET.");
      break;
    }
  }
  return params;
}

absl::StatusOr<CartesianImpedanceParameters> FromProto(
    const intrinsic_proto::icon::actions::proto::CartesianImpedanceParameters&
        proto_params) {
  INTRINSIC_ASSERT_NON_REALTIME();
  CartesianImpedanceParameters params;

  INTR_ASSIGN_OR_RETURN(params.cartesian_target,
                        FromProto(proto_params.cartesian_target()));
  INTR_ASSIGN_OR_RETURN(params.nullspace_target,
                        FromProto(proto_params.nullspace_target()));
  INTR_ASSIGN_OR_RETURN(params.constraints,
                        FromProto(proto_params.constraints()));
  INTR_ASSIGN_OR_RETURN(params.algorithm_configuration,
                        FromProto(proto_params.algorithm_configuration()));
  INTR_ASSIGN_OR_RETURN(params.state_variable_configuration,
                        FromProto(proto_params.state_variable_configuration()));
  INTR_ASSIGN_OR_RETURN(params.reference_generators,
                        FromProto(proto_params.reference_generators()));

  return params;
}

}  // namespace intrinsic::icon
