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

#ifndef INTRINSIC_ICON_ACTIONS_CARTESIAN_IMPEDANCE_PARAMETERS_H_
#define INTRINSIC_ICON_ACTIONS_CARTESIAN_IMPEDANCE_PARAMETERS_H_

#include <optional>
#include <variant>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/cartesian_impedance.pb.h"
#include "intrinsic/icon/actions/cartesian_impedance_reference_generators.pb.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

namespace cartesian_impedance {

// Defaults for algorithmic properties.
static constexpr double kDefaultJacobianPseudoInverseDamping = 1e-3;
static constexpr double kDefaultTimeWindowForMaximumDisplacement = 1.0;

struct CartesianTarget {
  // Offset between robot tip and 'tool frame'.
  Pose3d robot_tip_t_robot_tool = Pose3d::Identity();
  // Task frame expressed w.r.t. the robot base frame.
  Pose3d robot_base_t_task = Pose3d::Identity();
  // Impedance task parameters, all defined in the task frame.
  std::optional<Pose3d> task_t_tool_reference_pose;
  eigenmath::Matrix6d cartesian_stiffness;
  eigenmath::Matrix6d virtual_cartesian_inertia;
  std::optional<eigenmath::Matrix6d> cartesian_damping;
  eigenmath::Matrix6d wrench_selection_matrix = eigenmath::Matrix6d::Identity();
  Twist tool_reference_twist = Twist::ZERO;
  Wrench tool_reference_wrench = Wrench::ZERO;
  Acceleration tool_reference_acceleration = Acceleration::ZERO;
};

struct NullspaceTarget {
  std::optional<eigenmath::VectorNd> joint_position_nullspace;
  std::optional<double> nullspace_stiffness;
  std::optional<double> nullspace_damping;
};

struct AlgorithmConfiguration {
  Wrench sensed_wrench_deadband = Wrench::ZERO;
  double jacobian_pinv_damping = kDefaultJacobianPseudoInverseDamping;
  eigenmath::Vector6d pose_error_integrator_gain = eigenmath::Vector6d::Zero();
  eigenmath::Vector6d pose_error_integrator_bound = eigenmath::Vector6d::Zero();
  double post_sensor_dynamic_load_multiplier = 0.0;
};

struct Constraints {
  std::optional<JointLimits> joint_limits;
  std::optional<CartesianLimits> cartesian_limits;
  bool enforce_cartesian_and_joint_limits = false;
};

struct StateVariableConfiguration {
  double position_error_threshold = 1e-3;
  double orientation_error_threshold = 1e-2;
  double translational_velocity_error_threshold = 1e-2;
  double angular_velocity_error_threshold = 1e-2;
  double joint_velocity_threshold = 1e-2;
  std::optional<eigenmath::Vector3d> translational_distance_along =
      std::nullopt;
  double time_window_for_maximum_displacement =
      kDefaultTimeWindowForMaximumDisplacement;
  double force_settled_band = 3.0;
  eigenmath::Vector3d force_direction_1_in_task = eigenmath::Vector3d::UnitX();
  eigenmath::Vector3d force_direction_2_in_task = eigenmath::Vector3d::UnitY();
  eigenmath::Vector3d force_direction_3_in_task = eigenmath::Vector3d::UnitZ();
  eigenmath::Vector3d torque_axis_1_in_task = eigenmath::Vector3d::UnitX();
  eigenmath::Vector3d torque_axis_2_in_task = eigenmath::Vector3d::UnitY();
  eigenmath::Vector3d torque_axis_3_in_task = eigenmath::Vector3d::UnitZ();
  eigenmath::Vector3d position_error_direction_1_in_task =
      eigenmath::Vector3d::UnitX();
  eigenmath::Vector3d position_error_direction_2_in_task =
      eigenmath::Vector3d::UnitY();
  eigenmath::Vector3d position_error_direction_3_in_task =
      eigenmath::Vector3d::UnitZ();
  eigenmath::Vector3d orientation_error_axis_1_in_task =
      eigenmath::Vector3d::UnitX();
  eigenmath::Vector3d orientation_error_axis_2_in_task =
      eigenmath::Vector3d::UnitY();
  eigenmath::Vector3d orientation_error_axis_3_in_task =
      eigenmath::Vector3d::UnitZ();
};

struct CartesianPositionReflexxesImpedanceGeneratorParameters {
  // TODO(b/190081664): add parameters.
};
struct CartesianVelocityReflexxesImpedanceGeneratorParameters {
  // TODO(b/190081664): add parameters.
};
struct SimpleImpedanceGeneratorParameters {
  double lowpass_filter_constant = 0.005;
};
struct QuinticSplineCartesianGeneratorParameters {
  double movement_duration_seconds = 10.0;
};
struct ReflexxesNullspaceReferenceGeneratorParameters {
  // TODO(b/190081664): add parameters.
};
struct SimpleNullspaceReferenceGeneratorParameters {
  double lowpass_filter_constant = 0.005;
};
struct QuinticSplineNullspaceReferenceGeneratorParameters {
  double movement_duration_seconds = 5.0;
};

struct ReferenceGenerators {
  std::variant<CartesianPositionReflexxesImpedanceGeneratorParameters,
               CartesianVelocityReflexxesImpedanceGeneratorParameters,
               SimpleImpedanceGeneratorParameters,
               QuinticSplineCartesianGeneratorParameters>
      cartesian_reference_generator = SimpleImpedanceGeneratorParameters();

  std::variant<ReflexxesNullspaceReferenceGeneratorParameters,
               SimpleNullspaceReferenceGeneratorParameters,
               QuinticSplineNullspaceReferenceGeneratorParameters>
      nullspace_reference_generator =
          SimpleNullspaceReferenceGeneratorParameters();
};

}  // namespace cartesian_impedance

struct CartesianImpedanceParameters {
  cartesian_impedance::CartesianTarget cartesian_target;
  cartesian_impedance::NullspaceTarget nullspace_target;
  cartesian_impedance::Constraints constraints;
  cartesian_impedance::AlgorithmConfiguration algorithm_configuration;
  cartesian_impedance::StateVariableConfiguration state_variable_configuration;
  cartesian_impedance::ReferenceGenerators reference_generators;
};

intrinsic_proto::icon::actions::proto::
    CartesianPositionReflexxesImpedanceReferenceGenerator
    ToProto(const cartesian_impedance::
                CartesianPositionReflexxesImpedanceGeneratorParameters& params);

cartesian_impedance::CartesianPositionReflexxesImpedanceGeneratorParameters
FromProto(const intrinsic_proto::icon::actions::proto::
              CartesianPositionReflexxesImpedanceReferenceGenerator& proto);

intrinsic_proto::icon::actions::proto::
    CartesianVelocityReflexxesImpedanceReferenceGenerator
    ToProto(const cartesian_impedance::
                CartesianVelocityReflexxesImpedanceGeneratorParameters& params);

cartesian_impedance::CartesianVelocityReflexxesImpedanceGeneratorParameters
FromProto(const intrinsic_proto::icon::actions::proto::
              CartesianVelocityReflexxesImpedanceReferenceGenerator& proto);

intrinsic_proto::icon::actions::proto::ReflexxesNullspaceReferenceGenerator
ToProto(
    const cartesian_impedance::ReflexxesNullspaceReferenceGeneratorParameters&
        params);

cartesian_impedance::ReflexxesNullspaceReferenceGeneratorParameters FromProto(
    const intrinsic_proto::icon::actions::proto::
        ReflexxesNullspaceReferenceGenerator& proto);

intrinsic_proto::icon::actions::proto::QuinticSplineCartesianReferenceGenerator
ToProto(const cartesian_impedance::QuinticSplineCartesianGeneratorParameters&
            params);

cartesian_impedance::QuinticSplineCartesianGeneratorParameters FromProto(
    const intrinsic_proto::icon::actions::proto::
        QuinticSplineCartesianReferenceGenerator& proto);

intrinsic_proto::icon::actions::proto::SimpleImpedanceReferenceGenerator
ToProto(const cartesian_impedance::SimpleImpedanceGeneratorParameters& params);

cartesian_impedance::SimpleImpedanceGeneratorParameters FromProto(
    const intrinsic_proto::icon::actions::proto::
        SimpleImpedanceReferenceGenerator& proto);

intrinsic_proto::icon::actions::proto::QuinticSplineNullspaceReferenceGenerator
ToProto(const cartesian_impedance::
            QuinticSplineNullspaceReferenceGeneratorParameters& params);

cartesian_impedance::QuinticSplineNullspaceReferenceGeneratorParameters
FromProto(const intrinsic_proto::icon::actions::proto::
              QuinticSplineNullspaceReferenceGenerator& proto);

intrinsic_proto::icon::actions::proto::SimpleNullspaceReferenceGenerator
ToProto(const cartesian_impedance::SimpleNullspaceReferenceGeneratorParameters&
            params);

cartesian_impedance::SimpleNullspaceReferenceGeneratorParameters FromProto(
    const intrinsic_proto::icon::actions::proto::
        SimpleNullspaceReferenceGenerator& proto);

intrinsic_proto::icon::actions::proto::CartesianTarget ToProto(
    const cartesian_impedance::CartesianTarget& params);

intrinsic_proto::icon::actions::proto::NullspaceTarget ToProto(
    const cartesian_impedance::NullspaceTarget& params);

intrinsic_proto::icon::actions::proto::AlgorithmConfiguration ToProto(
    const cartesian_impedance::AlgorithmConfiguration& params);

intrinsic_proto::icon::actions::proto::Constraints ToProto(
    const cartesian_impedance::Constraints& proto_params);

intrinsic_proto::icon::actions::proto::StateVariableConfiguration ToProto(
    const cartesian_impedance::StateVariableConfiguration& proto_params);

intrinsic_proto::icon::actions::proto::ReferenceGenerators ToProto(
    const cartesian_impedance::ReferenceGenerators& proto_params);

intrinsic_proto::icon::actions::proto::CartesianImpedanceParameters ToProto(
    const CartesianImpedanceParameters& params);

absl::StatusOr<cartesian_impedance::CartesianTarget> FromProto(
    const intrinsic_proto::icon::actions::proto::CartesianTarget& proto_params);

absl::StatusOr<cartesian_impedance::NullspaceTarget> FromProto(
    const intrinsic_proto::icon::actions::proto::NullspaceTarget& proto_params);

absl::StatusOr<cartesian_impedance::AlgorithmConfiguration> FromProto(
    const intrinsic_proto::icon::actions::proto::AlgorithmConfiguration&
        proto_params);

absl::StatusOr<cartesian_impedance::Constraints> FromProto(
    const intrinsic_proto::icon::actions::proto::Constraints& proto_params);

absl::StatusOr<cartesian_impedance::StateVariableConfiguration> FromProto(
    const intrinsic_proto::icon::actions::proto::StateVariableConfiguration&
        proto_params);

absl::StatusOr<cartesian_impedance::ReferenceGenerators> FromProto(
    const intrinsic_proto::icon::actions::proto::ReferenceGenerators&
        proto_params);

absl::StatusOr<CartesianImpedanceParameters> FromProto(
    const intrinsic_proto::icon::actions::proto::CartesianImpedanceParameters&
        proto_params);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_ACTIONS_CARTESIAN_IMPEDANCE_PARAMETERS_H_
