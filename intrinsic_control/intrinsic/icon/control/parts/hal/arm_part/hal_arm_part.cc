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

#include "intrinsic/icon/control/parts/hal/arm_part/hal_arm_part.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "intrinsic/icon/control/collision/robot_collision_checker.h"
#include "intrinsic/icon/control/context.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/cartesian_limits.h"
#include "intrinsic/icon/control/parts/feature_interfaces/control_mode_state.h"
#include "intrinsic/icon/control/parts/feature_interfaces/handguiding_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces/homing.h"
#include "intrinsic/icon/control/parts/feature_interfaces/joint_acceleration_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces/joint_acceleration_estimated_state_feature.h"
#include "intrinsic/icon/control/parts/feature_interfaces/joint_acceleration_state.h"
#include "intrinsic/icon/control/parts/feature_interfaces/joint_limits.h"
#include "intrinsic/icon/control/parts/feature_interfaces/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces/joint_position_pid_torque_controller_feature.h"
#include "intrinsic/icon/control/parts/feature_interfaces/joint_position_state.h"
#include "intrinsic/icon/control/parts/feature_interfaces/joint_torque_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces/joint_torque_state.h"
#include "intrinsic/icon/control/parts/feature_interfaces/joint_velocity_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces/joint_velocity_state.h"
#include "intrinsic/icon/control/parts/feature_interfaces/move_ok.h"
#include "intrinsic/icon/control/parts/feature_interfaces/payload_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces/payload_state.h"
#include "intrinsic/icon/control/parts/feature_interfaces/process_wrench_at_endeffector_feature.h"
#include "intrinsic/icon/control/parts/hal/arm_part/hal_arm_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal/v1/hal_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal_realtime_part_base.h"
#include "intrinsic/icon/control/parts/new_manipulator_kinematics.h"
#include "intrinsic/icon/control/parts/payload_property.h"
#include "intrinsic/icon/control/parts/realtime_part_factory_common.h"
#include "intrinsic/icon/control/realtime_collision_world_from_world.h"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/control/services/assembly_service.h"
#include "intrinsic/icon/control/services/dynamics_service.h"
#include "intrinsic/icon/control/services/geometry_library_service.h"
#include "intrinsic/icon/control/services/kinematics_service.h"
#include "intrinsic/icon/control/services/world_service.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/flatbuffers/transform_types.fbs.h"
#include "intrinsic/icon/hal/default_hardware_interfaces.h"  // IWYU pragma: keep
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/hal/interfaces/control_mode.fbs.h"
#include "intrinsic/icon/hal/interfaces/electrical_motor.fbs.h"
#include "intrinsic/icon/hal/interfaces/electrical_motor_hardware_interfaces.h"  // IWYU pragma: keep
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_limits.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_state.fbs.h"
#include "intrinsic/icon/proto/cart_space.pb.h"
#include "intrinsic/icon/proto/cart_space_conversion.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/safety_status.pb.h"
#include "intrinsic/icon/proto/safety_status_conversion.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/elements.h"
#include "intrinsic/kinematics/ik/inverse_kinematics_interface.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/kinematic_object.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/robot_payload/robot_payload.h"

namespace intrinsic::icon {

namespace {

using ::intrinsic_fbs::EnumNameModeOfSafeOperation;
using ::intrinsic_fbs::EnumValuesModeOfSafeOperation;

absl::StatusOr<intrinsic_proto::icon::GenericPartConfig>
ExtractGenericConfigWithSafetyLimits(const HalArmPart& part,
                                     absl::string_view part_name) {
  intrinsic_proto::icon::GenericPartConfig generic_config =
      ExtractGenericConfig(part.GetFeatureInterfaces());

  // Covers all cases of ModeOfSafeOperation in
  // intrinsic/icon/control/safety/safety_messages.fbs

  auto& feature_interfaces = part.GetFeatureInterfaces();
  auto* joint_limits_interface =
      feature_interfaces.GetInterface<JointLimitsInterface>();
  auto* cartesian_limits_interface =
      feature_interfaces.GetInterface<CartesianLimitsInterface>();

  // Dynamic cast needed to access the limits map from the feature.
  auto* joint_limit_feature =
      dynamic_cast<const JointLimitsFeature*>(joint_limits_interface);
  auto* cartesian_limit_feature =
      dynamic_cast<const CartesianLimitsFeature*>(cartesian_limits_interface);

  if (joint_limit_feature != nullptr && cartesian_limit_feature != nullptr) {
    ::intrinsic_proto::icon::GenericSafetyLimitsConfig* safety_limits_config =
        generic_config.mutable_safety_limits_config();
    for (const auto mode : EnumValuesModeOfSafeOperation()) {
      LOG(INFO) << "Got LimitBundle for ModeOfSafeOperation::"
                << EnumNameModeOfSafeOperation(mode);
      intrinsic_proto::icon::GenericJointLimitsConfig
          generic_joint_limits_config;

      *generic_joint_limits_config.mutable_application_limits() = ToProto(
          joint_limit_feature->GetLimitBundleForModeOfSafeOperation(mode)
              .application_limits);
      *generic_joint_limits_config.mutable_system_limits() = ToProto(
          joint_limit_feature->GetLimitBundleForModeOfSafeOperation(mode)
              .system_limits);

      // Using the name as key for the map, as the Proto format doesn't allow
      // using an enum as key.
      std::string mode_proto_name =
          intrinsic_proto::icon::ModeOfSafeOperation_Name(ToProto(mode));

      (*safety_limits_config->mutable_joint_limits_map())[mode_proto_name] =
          generic_joint_limits_config;

      intrinsic_proto::icon::GenericCartesianLimitsConfig
          generic_cart_limits_config;
      *generic_cart_limits_config.mutable_default_cartesian_limits() = ToProto(
          cartesian_limit_feature->GetLimitsForModeOfSafeOperation(mode));
      (*safety_limits_config->mutable_cartesian_limits_map())[mode_proto_name] =
          generic_cart_limits_config;
    }
    // MODE_OF_SAFE_OPERATION_UNKNOWN is the fallback set of limits and needs to
    // be present.
    const std::string mode_unknown =
        intrinsic_proto::icon::ModeOfSafeOperation_Name(
            intrinsic_proto::icon::MODE_OF_SAFE_OPERATION_UNKNOWN);
    if (!safety_limits_config->joint_limits_map().contains(mode_unknown)) {
      return absl::InternalError(absl::StrCat(
          "Generic config for part '", part_name,
          "' does not contain joint_limits for '", mode_unknown, "'."));
    }
    if (!safety_limits_config->cartesian_limits_map().contains(mode_unknown)) {
      return absl::InternalError(absl::StrCat(
          "Generic config for part '", part_name,
          "' does not contain cartesian_limits for '", mode_unknown, "'."));
    }
  }
  return generic_config;
}

struct KinematicsInterfaces {
  std::unique_ptr<kinematics::InverseKinematicsInterface> ik_interface;
  std::unique_ptr<kinematics::Skeleton> kinematics_model;
};

// Tries to get both an IK interface and kinematics model (skeleton) for
// `model_name` from `kinematics_service`. `part_name` is just for nicer error
// messages.
absl::StatusOr<KinematicsInterfaces> GetKinematicsInterfaces(
    KinematicsService& kinematics_service, absl::string_view model_name,
    absl::string_view part_name) {
  KinematicsInterfaces kinematics_interfaces;

  kinematics_interfaces.ik_interface =
      kinematics_service.CreateInverseKinematicsSolverForPart(model_name);
  if (kinematics_interfaces.ik_interface == nullptr) {
    return absl::NotFoundError(absl::StrCat(
        "HalArmPart '", part_name,
        "' is configured to use a kinematics model, but the "
        "KinematicsService does not have an IK interface for a model named '",
        model_name, "'."));
  }
  kinematics_interfaces.kinematics_model =
      kinematics_service.CreateKinematicsModelForPart(model_name);

  if (kinematics_interfaces.kinematics_model == nullptr) {
    return absl::NotFoundError(absl::StrCat(
        "HalArmPart '", part_name,
        "' is configured to use a kinematics model, but the "
        "KinematicsService does not have a kinematics model named '",
        model_name, "'."));
  }

  return kinematics_interfaces;
}

absl::StatusOr<std::unique_ptr<RigidBodyInterface>> GetDynamicsInterface(
    absl::string_view part_name, absl::string_view hardware_resource_name,
    const intrinsic_proto::icon::HalArmPartConfig& proto_config,
    DynamicsService* dynamics_service_ptr) {
  std::unique_ptr<RigidBodyInterface> dynamics_interface = nullptr;
  if (dynamics_service_ptr != nullptr && !hardware_resource_name.empty()) {
    INTRINSIC_RT_LOG(INFO) << "Creating dynamics solver for "
                           << hardware_resource_name;
    absl::StatusOr<std::unique_ptr<RigidBodyInterface>> dynamics_interface_or =
        dynamics_service_ptr->CreateDynamicsSolverForPart(
            hardware_resource_name);
    if (dynamics_interface_or.ok()) {
      dynamics_interface = std::move(dynamics_interface_or.value());
    }
  }
  if (dynamics_interface == nullptr && proto_config.has_dynamics_model_name()) {
    if (dynamics_service_ptr == nullptr) {
      return absl::NotFoundError(
          absl::StrCat("HalArmPart '", part_name,
                       "' is configured to use a dynamics model, but there "
                       "is no DynamicsService."));
    }
    INTRINSIC_RT_LOG(INFO) << "Creating dynamics solver from config for "
                           << proto_config.dynamics_model_name();
    INTR_ASSIGN_OR_RETURN(dynamics_interface,
                          dynamics_service_ptr->CreateDynamicsSolverForPart(
                              proto_config.dynamics_model_name()));
  }
  return std::move(dynamics_interface);
}

}  // namespace

HalArmPart::HalArmPart(absl::string_view name,
                       HardwareModuleManager* const manager)
    : HalRealtimePartBase(manager), name_(name) {}

template <class HardwareInterfaceT, class FeatureT, class... AdditionalParamsTs>
absl::StatusOr<FeatureT> MapHardwareInterfaceToFeatureInterface(
    const Context& context,
    const intrinsic_proto::icon::v1::HardwareInterface& hardware_interface,
    AdditionalParamsTs... additional_params) {
  INTR_ASSIGN_OR_RETURN(auto hardware_handle,
                        context.GetHardwareInterfaceHandle<HardwareInterfaceT>(
                            hardware_interface.module_name(),
                            hardware_interface.interface_name()));
  return FeatureT::Create(std::move(hardware_handle),
                          std::move(additional_params)...);
}

template <class HardwareInterfaceT, class FeatureT, class... AdditionalParamsTs>
absl::StatusOr<FeatureT> MapMutableHardwareInterfaceToFeatureInterface(
    const Context& context,
    const intrinsic_proto::icon::v1::HardwareInterface& hardware_interface,
    AdditionalParamsTs... additional_params) {
  INTR_ASSIGN_OR_RETURN(
      auto hardware_handle,
      context.GetMutableHardwareInterfaceHandle<HardwareInterfaceT>(
          hardware_interface.module_name(),
          hardware_interface.interface_name()));
  return FeatureT::Create(std::move(hardware_handle),
                          std::move(additional_params)...);
}

namespace {

template <typename ProtoT>
  requires std::is_same_v<decltype(std::declval<ProtoT>().module_name()),
                          const std::string&> &&
           std::is_same_v<decltype(std::declval<ProtoT>().interface_name()),
                          const std::string&>
bool IsNonEmptyHardwareInterfaceConfig(const ProtoT& proto) {
  return !proto.module_name().empty() && !proto.interface_name().empty();
}

}  // namespace

// static
absl::StatusOr<PartPtrAndGenericConfig> HalArmPart::FromProto(
    PartFactoryContext context,
    const intrinsic_proto::icon::HalArmPartConfig& proto_config) {
  auto part = std::make_unique<HalArmPart>(
      context.part_name, context.context.GetHardwareModuleManager());

  LOG(INFO) << "Instantiating hal arm part with hardware resource name: "
            << context.hardware_resource_name;
  // We analyze the hal arm part proto and register each listed hardware
  // interface to its respective feature interface.
  // All hardware interfaces are optional, meaning we only add the feature
  // interfaces which are explicitly set in the pbtxt.

  bool has_joint_position_command =
      proto_config.has_joint_position_command() &&
      IsNonEmptyHardwareInterfaceConfig(proto_config.joint_position_command());
  bool has_joint_commanded_position =
      proto_config.has_joint_commanded_position() &&
      IsNonEmptyHardwareInterfaceConfig(
          proto_config.joint_commanded_position());
  bool has_joint_velocity_command =
      proto_config.has_joint_velocity_command() &&
      IsNonEmptyHardwareInterfaceConfig(proto_config.joint_velocity_command());
  bool has_joint_acceleration_command =
      proto_config.has_joint_acceleration_command() &&
      IsNonEmptyHardwareInterfaceConfig(
          proto_config.joint_acceleration_command());
  bool has_joint_torque_command =
      proto_config.has_joint_torque_command() &&
      IsNonEmptyHardwareInterfaceConfig(proto_config.joint_torque_command());
  bool has_joint_position_state =
      proto_config.has_joint_position_state() &&
      IsNonEmptyHardwareInterfaceConfig(proto_config.joint_position_state());
  bool has_joint_velocity_state =
      proto_config.has_joint_velocity_state() &&
      IsNonEmptyHardwareInterfaceConfig(proto_config.joint_velocity_state());
  bool has_joint_acceleration_state =
      proto_config.has_joint_acceleration_state() &&
      IsNonEmptyHardwareInterfaceConfig(
          proto_config.joint_acceleration_state());
  bool has_joint_torque_state =
      proto_config.has_joint_torque_state() &&
      IsNonEmptyHardwareInterfaceConfig(proto_config.joint_torque_state());
  bool has_process_wrench_command =
      proto_config.has_process_wrench_command() &&
      IsNonEmptyHardwareInterfaceConfig(proto_config.process_wrench_command());
  bool has_hand_guiding_command =
      proto_config.has_hand_guiding_command() &&
      IsNonEmptyHardwareInterfaceConfig(proto_config.hand_guiding_command());
  bool has_control_mode_state =
      proto_config.has_control_mode_state() &&
      IsNonEmptyHardwareInterfaceConfig(proto_config.control_mode_state());
  bool has_payload_command =
      proto_config.has_payload_command() &&
      IsNonEmptyHardwareInterfaceConfig(proto_config.payload_command());
  bool has_payload_state =
      proto_config.has_payload_state() &&
      IsNonEmptyHardwareInterfaceConfig(proto_config.payload_state());
  bool has_joint_system_limits =
      proto_config.has_joint_system_limits() &&
      IsNonEmptyHardwareInterfaceConfig(proto_config.joint_system_limits());

  if (has_process_wrench_command) {
    INTR_ASSIGN_OR_RETURN(
        auto process_wrench_feature_interface,
        (MapMutableHardwareInterfaceToFeatureInterface<
            intrinsic_fbs::Wrench, ProcessWrenchAtEndeffectorFeature>(
            context.context, proto_config.process_wrench_command())),
        _ << "for process_wrench_command");
    INTR_RETURN_IF_ERROR(
        part->RegisterInterface(std::move(process_wrench_feature_interface)));
  }

  auto* dynamics_service_ptr =
      context.context.GetServiceOrNull<DynamicsService>();
  if (!context.hardware_resource_name.empty() &&
      (has_joint_position_command || has_joint_velocity_command ||
       has_joint_torque_command)) {
    // Attempt to read joint system and application limits from World.
    const WorldService* world_service =
        context.context.GetServiceOrNull<WorldService>();
    if (world_service == nullptr) {
      return absl::NotFoundError(
          absl::Substitute("HalArmPart '$0' does not read limits from Hardware "
                           "Module, and there is no WorldService available "
                           "to read application limits from either.",
                           context.part_name));
    }

    std::shared_ptr<const world::ObjectWorldClient> world_client =
        world_service->GetObjectWorldClient();
    INTR_ASSIGN_OR_RETURN(world::KinematicObject kinematic_object,
                          world_client->GetKinematicObject(
                              WorldObjectName(context.hardware_resource_name)));
    // Need to convert these to proto and back, because the KinematicObject
    // returns `JointLimitsXd`, and they could in theory have too many DoFs to
    // represent in the realtime safe `JointLimits` class. The `FromProto()`
    // function checks for that.
    INTR_ASSIGN_OR_RETURN(JointLimits application_limits,
                          ::intrinsic::FromProto(ToProto(
                              kinematic_object.JointApplicationLimits())));
    INTR_ASSIGN_OR_RETURN(
        JointLimits system_limits,
        ::intrinsic::FromProto(ToProto(kinematic_object.JointSystemLimits())));

    intrinsic_proto::icon::v1::ModeOfSafeOperationLimitsConfig
        mode_of_safe_limits_config;
    if (proto_config.has_mode_of_safe_operation_limits_config()) {
      mode_of_safe_limits_config =
          proto_config.mode_of_safe_operation_limits_config();
    }
    INTR_ASSIGN_OR_RETURN(
        std::unique_ptr<RigidBodyInterface> dynamics_interface_for_joint_limits,
        GetDynamicsInterface(context.part_name, context.hardware_resource_name,
                             proto_config, dynamics_service_ptr));
    std::optional<HardwareInterfaceHandle<intrinsic_fbs::JointPositionCommand>>
        joint_position_command_hardware_interface = std::nullopt;
    if (has_joint_position_command) {
      INTR_ASSIGN_OR_RETURN(
          joint_position_command_hardware_interface,
          context.context
              .GetHardwareInterfaceHandle<intrinsic_fbs::JointPositionCommand>(
                  proto_config.joint_position_command().module_name(),
                  proto_config.joint_position_command().interface_name()),
          _ << "for joint_position_command");
      INTR_RETURN_IF_ERROR(part->AddHardwareModule(
          proto_config.joint_position_command().module_name()));
    }

    std::optional<MutableHardwareInterfaceHandle<intrinsic_fbs::JointLimits>>
        joint_system_limits_hardware_interface = std::nullopt;
    if (has_joint_system_limits) {
      INTR_ASSIGN_OR_RETURN(
          joint_system_limits_hardware_interface,
          context.context
              .GetMutableHardwareInterfaceHandle<intrinsic_fbs::JointLimits>(
                  proto_config.joint_system_limits().module_name(),
                  proto_config.joint_system_limits().interface_name()),
          _ << "for joint_system_limits");
      INTR_RETURN_IF_ERROR(part->AddHardwareModule(
          proto_config.joint_system_limits().module_name()));
    }

    INTR_ASSIGN_OR_RETURN(
        auto joint_limits_feature_interface,
        JointLimitsFeature::Create(
            /*system_limits=*/system_limits,
            /*application_limits=*/application_limits,
            /*config=*/mode_of_safe_limits_config,
            std::move(joint_position_command_hardware_interface),
            std::move(dynamics_interface_for_joint_limits),
            std::move(joint_system_limits_hardware_interface)));
    // Given that the joint_limits_feature_interface depends on
    // joint_position_command_hardware_interface, they should respect the order
    // joint_position_command_hardware_interface before
    // joint_limits_feature_interface when registering the interfaces.
    INTR_RETURN_IF_ERROR(
        part->RegisterInterface(std::move(joint_limits_feature_interface)));
  }

  if (proto_config.has_linear_joint_acceleration_filter_config() &&
      has_joint_acceleration_state) {
    return absl::InvalidArgumentError(
        "Cannot define both `joint_acceleration_state` and "
        "`linear_joint_acceleration_filter_config` in the HalArmPart "
        "config.");
  }
  if (has_joint_acceleration_state) {
    INTR_ASSIGN_OR_RETURN(
        auto joint_acceleration_state_feature_interface,
        (MapHardwareInterfaceToFeatureInterface<
            intrinsic_fbs::JointAccelerationState,
            JointAccelerationStateFeature>(
            context.context, proto_config.joint_acceleration_state())),
        _ << "for joint_acceleration_state");
    INTR_RETURN_IF_ERROR(part->RegisterInterface(
        std::move(joint_acceleration_state_feature_interface)));
    INTR_RETURN_IF_ERROR(part->AddHardwareModule(
        proto_config.joint_acceleration_state().module_name()));
  }
  if (proto_config.has_linear_joint_acceleration_filter_config()) {
    if (!has_joint_position_state) {
      return absl::FailedPreconditionError(
          "Cannot define `linear_joint_acceleration_filter` without "
          "`joint_position_state`.");
    }
    if (!has_joint_velocity_state) {
      return absl::FailedPreconditionError(
          "Cannot define `linear_joint_acceleration_filter` without "
          "`joint_velocity_state`.");
    }
    INTR_ASSIGN_OR_RETURN(
        auto position_hardware_handle,
        context.context
            .GetHardwareInterfaceHandle<intrinsic_fbs::JointPositionState>(
                proto_config.joint_position_state().module_name(),
                proto_config.joint_position_state().interface_name()),
        _ << "for joint_position_state");
    INTR_ASSIGN_OR_RETURN(
        auto velocity_hardware_handle,
        context.context
            .GetHardwareInterfaceHandle<intrinsic_fbs::JointVelocityState>(
                proto_config.joint_velocity_state().module_name(),
                proto_config.joint_velocity_state().interface_name()),
        _ << "for joint_velocity_state");
    INTR_ASSIGN_OR_RETURN(
        auto joint_acceleration_estimated_state_feature_interface,
        JointAccelerationEstimatedStateFeature::Create(
            std::move(position_hardware_handle),
            std::move(velocity_hardware_handle),
            proto_config.linear_joint_acceleration_filter_config(),
            context.control_frequency_hz));
    INTR_RETURN_IF_ERROR(part->RegisterInterface(
        std::move(joint_acceleration_estimated_state_feature_interface)));
  }

  if (has_joint_torque_state) {
    INTR_ASSIGN_OR_RETURN(
        auto joint_torque_state_feature_interface,
        (MapHardwareInterfaceToFeatureInterface<intrinsic_fbs::JointTorqueState,
                                                JointTorqueStateFeature>(
            context.context, proto_config.joint_torque_state())),
        _ << "for joint_torque_state");
    INTR_RETURN_IF_ERROR(part->RegisterInterface(
        std::move(joint_torque_state_feature_interface)));
    INTR_RETURN_IF_ERROR(part->AddHardwareModule(
        proto_config.joint_torque_state().module_name()));
  }

  auto fetch_cartesian_limits_from_world =
      [&context]() -> absl::StatusOr<CartesianLimits> {
    const WorldService* world_service =
        context.context.GetServiceOrNull<WorldService>();
    if (world_service == nullptr) {
      return absl::NotFoundError(absl::Substitute(
          "HalArmPart '$0' is attempting to fetch Cartesian limits from the "
          "world, but there's no world service available",
          context.part_name));
    }
    std::shared_ptr<const world::ObjectWorldClient> world_client =
        world_service->GetObjectWorldClient();
    INTR_ASSIGN_OR_RETURN(world::KinematicObject kinematic_object,
                          world_client->GetKinematicObject(
                              WorldObjectName(context.hardware_resource_name)));
    INTR_ASSIGN_OR_RETURN(CartesianLimits cart_limits,
                          kinematic_object.GetCartesianLimits());
    if (!cart_limits.IsValid()) {
      return absl::InternalError(
          "Invalid Cartesian Limits specified in the world.");
    }
    return cart_limits;
  };

  auto maybe_cartesian_limits = fetch_cartesian_limits_from_world();
  if (maybe_cartesian_limits.ok()) {
    LOG(INFO) << "Registering Cartesian limits interface from world.";
    INTR_ASSIGN_OR_RETURN(
        auto cartesian_limits_feature,
        CartesianLimitsFeature::Create(
            std::move(*maybe_cartesian_limits),
            proto_config.mode_of_safe_operation_limits_config()));
    INTR_RETURN_IF_ERROR(
        part->RegisterInterface(std::move(cartesian_limits_feature)));
  } else if (proto_config.has_mode_of_safe_operation_limits_config()) {
    LOG(INFO) << "Registering Cartesian limits interface from safety limits.";
    INTR_ASSIGN_OR_RETURN(
        auto cartesian_limits_feature,
        CartesianLimitsFeature::Create(
            proto_config.mode_of_safe_operation_limits_config()));
    INTR_RETURN_IF_ERROR(
        part->RegisterInterface(std::move(cartesian_limits_feature)));
  } else {
    LOG(ERROR) << "Unable to register Cartesian limits interface: "
               << maybe_cartesian_limits.status().message();
  }

  std::unique_ptr<kinematics::InverseKinematicsInterface>
      inverse_kinematics_interface;
  std::unique_ptr<kinematics::Skeleton> kinematics_model;
  // Retrieve kinematics service and (if available) the kinematics
  // implementations for this part.
  auto* kinematics_service_ptr =
      context.context.GetServiceOrNull<KinematicsService>();
  if (!context.hardware_resource_name.empty()) {
    if (kinematics_service_ptr == nullptr) {
      LOG(ERROR) << "HalArmPart: There is no kinematics service. This is "
                    "unusual, maybe there is a configuration error?";
    } else {
      absl::StatusOr<KinematicsInterfaces> kinematics_interfaces =
          GetKinematicsInterfaces(*kinematics_service_ptr,
                                  context.hardware_resource_name,
                                  context.part_name);
      if (kinematics_interfaces.ok()) {
        inverse_kinematics_interface =
            std::move(kinematics_interfaces->ik_interface);
        kinematics_model = std::move(kinematics_interfaces->kinematics_model);
      }
    }
  }
  if ((inverse_kinematics_interface == nullptr ||
       kinematics_model == nullptr) &&
      proto_config.has_kinematics_model_name()) {
    if (kinematics_service_ptr == nullptr) {
      return absl::NotFoundError(
          absl::StrCat("HalArmPart '", context.part_name,
                       "' is configured to use a kinematics model, but there "
                       "is no KinematicsService."));
    }

    inverse_kinematics_interface =
        kinematics_service_ptr->CreateInverseKinematicsSolverForPart(
            proto_config.kinematics_model_name());
    if (inverse_kinematics_interface == nullptr) {
      return absl::NotFoundError(absl::StrCat(
          "HalArmPart '", context.part_name,
          "' is configured to use a kinematics model, but the "
          "KinematicsService does not have an IK interface for a model named '",
          proto_config.kinematics_model_name(), "'."));
    }
    kinematics_model = kinematics_service_ptr->CreateKinematicsModelForPart(
        proto_config.kinematics_model_name());

    if (kinematics_model == nullptr) {
      return absl::NotFoundError(absl::StrCat(
          "HalArmPart '", context.part_name,
          "' is configured to use a kinematics model, but the "
          "KinematicsService does not have a kinematics model named '",
          proto_config.kinematics_model_name(), "'."));
    }
    if (!part->GetFeatureInterfaces()
             .GetInterface<CartesianLimitsInterface>()) {
      return absl::FailedPreconditionError(absl::StrCat(
          "HAL Arm Part '", context.part_name, " for RTCL HALArmPart '",
          context.part_name, "' has kinematics, but no Cartesian Limits."));
    }
  }
  if (inverse_kinematics_interface != nullptr && kinematics_model != nullptr) {
    auto kinematics = std::make_unique<NewManipulatorKinematicsImpl>(
        std::move(inverse_kinematics_interface), std::move(kinematics_model));
    CHECK_OK(part->RegisterManipulatorInterface(std::move(kinematics)));
  }

  if (has_joint_position_command &&
      proto_config.has_arm_position_pid_torque_controller_config()) {
    return absl::InvalidArgumentError(
        "Cannot define both a `joint_position_command` and "
        "`arm_position_pid_torque_controller_config`.");
  }
  if (proto_config.has_arm_position_pid_torque_controller_config()) {
    if (!has_joint_position_state) {
      return absl::NotFoundError(
          "JointPositionPidTorqueControllerFeature requires "
          "JointPositionState");
    }
    INTR_ASSIGN_OR_RETURN(
        auto position_hardware_handle,
        context.context
            .GetHardwareInterfaceHandle<intrinsic_fbs::JointPositionState>(
                proto_config.joint_position_state().module_name(),
                proto_config.joint_position_state().interface_name()),
        _ << "for joint_position_state");
    INTR_ASSIGN_OR_RETURN(
        auto velocity_hardware_handle,
        context.context
            .GetHardwareInterfaceHandle<intrinsic_fbs::JointVelocityState>(
                proto_config.joint_velocity_state().module_name(),
                proto_config.joint_velocity_state().interface_name()),
        _ << "for joint_velocity_state");
    INTR_ASSIGN_OR_RETURN(
        auto torque_command_handle,
        context.context.GetMutableHardwareInterfaceHandle<
            intrinsic_fbs::JointTorqueCommand>(
            proto_config.joint_torque_command().module_name(),
            proto_config.joint_torque_command().interface_name()),
        _ << "for joint_torque_command");
    const auto* limits_interface =
        part->GetFeatureInterfaces().GetInterface<JointLimitsInterface>();
    if (limits_interface == nullptr) {
      return absl::NotFoundError(
          "JointPositionPidTorqueControllerFeature requires JointLimits");
    }
    INTR_ASSIGN_OR_RETURN(
        auto joint_position_pid_torque_controller_feature_interface,
        JointPositionPidTorqueControllerFeature::Create(
            std::move(torque_command_handle),
            std::move(position_hardware_handle),
            std::move(velocity_hardware_handle),
            proto_config.arm_position_pid_torque_controller_config(),
            context.control_frequency_hz, limits_interface));
    INTR_RETURN_IF_ERROR(part->RegisterInterface(
        std::move(joint_position_pid_torque_controller_feature_interface)));
  }
  if (has_joint_position_command) {
    if (!has_joint_position_state) {
      return absl::NotFoundError(
          "JointPositionCommand requires JointPositionState");
    }
    const auto* limits_interface =
        part->GetFeatureInterfaces().GetInterface<JointLimitsInterface>();
    if (limits_interface == nullptr) {
      return absl::NotFoundError("JointPositionCommand requires JointLimits");
    }
    INTR_ASSIGN_OR_RETURN(
        auto joint_position_state_handle,
        context.context
            .GetHardwareInterfaceHandle<intrinsic_fbs::JointPositionState>(
                proto_config.joint_position_state().module_name(),
                proto_config.joint_position_state().interface_name()),
        _ << "for joint_position_state");
    std::optional<intrinsic::icon::HardwareInterfaceHandle<
        intrinsic_fbs::JointCommandedPosition>>
        joint_commanded_position_status_handle;
    if (has_joint_commanded_position) {
      INTR_ASSIGN_OR_RETURN(
          joint_commanded_position_status_handle,
          context.context.GetHardwareInterfaceHandle<
              intrinsic_fbs::JointCommandedPosition>(
              proto_config.joint_commanded_position().module_name(),
              proto_config.joint_commanded_position().interface_name()),
          _ << "for joint_commanded_position");
    }
    ManipulatorKinematics* manipulator_kinematics_interface = nullptr;
    if (proto_config.has_kinematics_model_name()) {
      manipulator_kinematics_interface =
          part->GetFeatureInterfaces().GetInterface<ManipulatorKinematics>();
      if (manipulator_kinematics_interface == nullptr) {
        return absl::NotFoundError(
            "Could not get ManipulatorKinematics when creating "
            "JointPositionCommand.");
      }
    }

    std::unique_ptr<collision::RobotCollisionChecker> collision_checker =
        nullptr;
    if (proto_config.has_check_collisions() &&
        proto_config.check_collisions()) {
      WorldService* world_service =
          context.context.GetServiceOrNull<WorldService>();
      if (world_service == nullptr) {
        return absl::FailedPreconditionError(
            "Collision checking is enabled but WorldService is not available.");
      }
      AssemblyService* assembly_service =
          context.context.GetServiceOrNull<AssemblyService>();
      if (assembly_service == nullptr) {
        return absl::FailedPreconditionError(
            "Collision checking is enabled but AssemblyService is not "
            "available.");
      }
      const kinematics::Skeleton* skeleton =
          assembly_service->GetSkeletonOrNull();
      if (skeleton == nullptr) {
        return absl::FailedPreconditionError(
            "AssemblyService does not provide a Skeleton, cannot initialize "
            "collision checker.");
      }
      GeometryLibraryService* geo_service =
          context.context.GetServiceOrNull<GeometryLibraryService>();
      if (geo_service == nullptr) {
        return absl::FailedPreconditionError(
            "Collision checking is enabled but GeometryLibraryService is not "
            "available. Cannot initialize collision world from world.");
      }
      INTR_ASSIGN_OR_RETURN(
          icon::CollisionWorldWithLinkNames collision_world_and_link_names,
          icon::RealtimeCollisionWorldFromWorld(
              *(world_service->GetObjectWorldClient()),
              geo_service->GetGeometryLibrary()->Deserializer(),
              /*enable_environment_collision=*/false));
      collision_checker = std::make_unique<collision::RobotCollisionChecker>();
      INTRINSIC_RT_RETURN_IF_ERROR(collision_checker->Init(
          // Make a copy of the skeleton for collision_checker_ to own.
          skeleton->Clone(),
          std::move(collision_world_and_link_names.collision_world),
          std::move(collision_world_and_link_names.link_names)));
    }

    auto* cart_limit_feature_interface =
        part->GetFeatureInterfaces().GetInterface<CartesianLimitsInterface>();
    auto cartesian_limits =
        cart_limit_feature_interface
            ? std::make_optional(
                  cart_limit_feature_interface->GetDefaultCartesianLimits())
            : std::nullopt;

    INTR_ASSIGN_OR_RETURN(
        auto joint_position_command_feature_interface,
        (MapMutableHardwareInterfaceToFeatureInterface<
            intrinsic_fbs::JointPositionCommand, JointPositionCommandFeature>(
            context.context, proto_config.joint_position_command(),
            std::move(joint_position_state_handle),
            std::move(joint_commanded_position_status_handle),
            context.control_frequency_hz, limits_interface, cartesian_limits,
            manipulator_kinematics_interface, std::move(collision_checker))),
        _ << "for joint_position_command");

    INTR_RETURN_IF_ERROR(part->RegisterInterface(
        std::move(joint_position_command_feature_interface)));
    INTR_RETURN_IF_ERROR(part->AddHardwareModule(
        proto_config.joint_position_command().module_name()));
  }
  if (has_joint_velocity_command) {
    INTR_ASSIGN_OR_RETURN(
        auto joint_velocity_command_feature_interface,
        (MapMutableHardwareInterfaceToFeatureInterface<
            intrinsic_fbs::JointVelocityCommand, JointVelocityCommandFeature>(
            context.context, proto_config.joint_velocity_command())),
        _ << "for joint_velocity_command");
    INTR_RETURN_IF_ERROR(part->RegisterInterface(
        std::move(joint_velocity_command_feature_interface)));
    INTR_RETURN_IF_ERROR(part->AddHardwareModule(
        proto_config.joint_velocity_command().module_name()));
  }
  if (has_joint_acceleration_command) {
    INTR_ASSIGN_OR_RETURN(
        auto joint_acceleration_command_feature_interface,
        (MapMutableHardwareInterfaceToFeatureInterface<
            intrinsic_fbs::JointAccelerationAndTorqueCommand,
            JointAccelerationCommandFeature>(
            context.context, proto_config.joint_acceleration_command())));
    INTR_RETURN_IF_ERROR(part->RegisterInterface(
        std::move(joint_acceleration_command_feature_interface)));
    INTR_RETURN_IF_ERROR(part->AddHardwareModule(
        proto_config.joint_acceleration_command().module_name()));
  }
  if (has_joint_torque_command) {
    INTR_ASSIGN_OR_RETURN(
        auto joint_torque_command_feature_interface,
        (MapMutableHardwareInterfaceToFeatureInterface<
            intrinsic_fbs::JointTorqueCommand, JointTorqueCommandFeature>(
            context.context, proto_config.joint_torque_command())),
        _ << "for joint_torque_command");
    INTR_RETURN_IF_ERROR(part->RegisterInterface(
        std::move(joint_torque_command_feature_interface)));
    INTR_RETURN_IF_ERROR(part->AddHardwareModule(
        proto_config.joint_torque_command().module_name()));
  }
  if (has_joint_position_state) {
    INTR_ASSIGN_OR_RETURN(
        auto joint_position_state_feature_interface,
        (MapHardwareInterfaceToFeatureInterface<
            intrinsic_fbs::JointPositionState, JointPositionStateFeature>(
            context.context, proto_config.joint_position_state())),
        _ << "for joint_position_state");
    INTR_RETURN_IF_ERROR(part->RegisterInterface(
        std::move(joint_position_state_feature_interface)));
    INTR_RETURN_IF_ERROR(part->AddHardwareModule(
        proto_config.joint_position_state().module_name()));
  }
  if (has_joint_velocity_state &&
      proto_config.has_calculate_velocity_state_from_position() &&
      proto_config.calculate_velocity_state_from_position()) {
    return absl::InvalidArgumentError(
        "Only one of joint_velocity_state and "
        "calculate_velocity_state_from_position can be set.");
  }
  if (has_joint_velocity_state) {
    if (proto_config.has_velocity_filter_cutoff_frequency()) {
      INTR_ASSIGN_OR_RETURN(
          auto joint_velocity_state_feature_interface,
          (MapHardwareInterfaceToFeatureInterface<
              intrinsic_fbs::JointVelocityState, JointVelocityStateFeature>(
              context.context, proto_config.joint_velocity_state(),
              context.control_frequency_hz,
              proto_config.velocity_filter_cutoff_frequency())),
          _ << "for joint_velocity_state");
      INTR_RETURN_IF_ERROR(part->RegisterInterface(
          std::move(joint_velocity_state_feature_interface)));
    } else {
      INTR_ASSIGN_OR_RETURN(
          auto joint_velocity_state_feature_interface,
          (MapHardwareInterfaceToFeatureInterface<
              intrinsic_fbs::JointVelocityState, JointVelocityStateFeature>(
              context.context, proto_config.joint_velocity_state())),
          _ << "for joint_velocity_state");
      INTR_RETURN_IF_ERROR(part->RegisterInterface(
          std::move(joint_velocity_state_feature_interface)));
    }
    INTR_RETURN_IF_ERROR(part->AddHardwareModule(
        proto_config.joint_velocity_state().module_name()));
  }
  if (has_joint_position_state &&
      proto_config.has_calculate_velocity_state_from_position() &&
      proto_config.calculate_velocity_state_from_position()) {
    if (proto_config.has_velocity_filter_cutoff_frequency()) {
      INTR_ASSIGN_OR_RETURN(
          auto joint_velocity_state_feature_interface,
          (MapHardwareInterfaceToFeatureInterface<
              intrinsic_fbs::JointPositionState, JointVelocityStateFeature>(
              context.context, proto_config.joint_position_state(),
              context.control_frequency_hz,
              proto_config.velocity_filter_cutoff_frequency())),
          _ << "for joint_position_state");
      INTR_RETURN_IF_ERROR(part->RegisterInterface(
          std::move(joint_velocity_state_feature_interface)));
    } else {
      INTR_ASSIGN_OR_RETURN(
          auto joint_velocity_state_feature_interface,
          (MapHardwareInterfaceToFeatureInterface<
              intrinsic_fbs::JointPositionState, JointVelocityStateFeature>(
              context.context, proto_config.joint_position_state(),
              context.control_frequency_hz)));
      INTR_RETURN_IF_ERROR(part->RegisterInterface(
          std::move(joint_velocity_state_feature_interface)))
          << "for joint_position_state";
    }
  }
  if (part->GetFeatureInterfaces().GetInterface<JointVelocityEstimator>() ==
      nullptr) {
    LOG(WARNING)
        << "No `JointVelocityEstimator` registered. Please configure a "
           "`joint_velocity_state` or set "
           "`calculate_velocity_state_from_position` to true.";
  }
  // MoveOk interface requires the existence of joint limits and is intended
  // for use with joint position commands.
  if (const auto [joint_limit_interface, joint_pos_interface] = std::make_tuple(
          part->GetFeatureInterfaces().GetInterface<JointLimitsInterface>(),
          part->GetFeatureInterfaces().GetInterface<JointPosition>());
      joint_limit_interface != nullptr && joint_pos_interface != nullptr) {
    INTR_ASSIGN_OR_RETURN(
        auto move_ok,
        MoveOkFeature::Create(context.control_frequency_hz,
                              joint_limit_interface->GetApplicationLimits(),
                              joint_limit_interface->GetSystemLimits(),
                              joint_pos_interface));
    CHECK_OK(part->RegisterInterface(std::move(move_ok)));
  }
  if (has_hand_guiding_command) {
    INTR_ASSIGN_OR_RETURN(
        auto hand_guiding_command_feature_interface,
        (MapMutableHardwareInterfaceToFeatureInterface<
            intrinsic_fbs::HandGuidingCommand, HandGuidingCommandFeature>(
            context.context, proto_config.hand_guiding_command())),
        _ << "for hand_guiding_command");
    INTR_RETURN_IF_ERROR(part->RegisterInterface(
        std::move(hand_guiding_command_feature_interface)));
    INTR_RETURN_IF_ERROR(part->AddHardwareModule(
        proto_config.hand_guiding_command().module_name()));
  }

  if (has_control_mode_state) {
    INTR_ASSIGN_OR_RETURN(
        auto control_mode_state_feature_interface,
        (MapHardwareInterfaceToFeatureInterface<
            intrinsic_fbs::ControlModeStatus, ControlModeStateFeature>(
            context.context, proto_config.control_mode_state())),
        _ << "for control_mode_state");
    INTR_RETURN_IF_ERROR(part->RegisterInterface(
        std::move(control_mode_state_feature_interface)));
    INTR_RETURN_IF_ERROR(part->AddHardwareModule(
        proto_config.control_mode_state().module_name()));
  }

  if (proto_config.homing_size() > 0) {
    absl::flat_hash_map<std::string, HomingFeature::HomingInterfaces>
        homing_interfaces;
    for (auto& [drive_name, homing] : proto_config.homing()) {
      INTR_ASSIGN_OR_RETURN(
          auto homing_command_interface,
          context.context
              .GetMutableHardwareInterfaceHandle<::intrinsic_fbs::HomeCommand>(
                  homing.command().module_name(),
                  homing.command().interface_name()),
          _ << "for homing command");

      INTR_ASSIGN_OR_RETURN(
          auto homing_state_interface,
          context.context
              .GetHardwareInterfaceHandle<::intrinsic_fbs::HomingStatus>(
                  homing.state().module_name(),
                  homing.state().interface_name()),
          _ << "for homing state");

      homing_interfaces.emplace(
          drive_name, HomingFeature::HomingInterfaces{
                          .command = std::move(homing_command_interface),
                          .status = std::move(homing_state_interface),
                      });
    };
    INTR_ASSIGN_OR_RETURN(auto homing_feature,
                          HomingFeature::Create(std::move(homing_interfaces)));
    INTR_RETURN_IF_ERROR(part->RegisterInterface(std::move(homing_feature)));
  }

  // The payload can be used in the payload command interface and also to set
  // the correct end effector inertia in the dynamics interface.
  auto fetch_payload_from_world =
      [&context]() -> absl::StatusOr<std::optional<RobotPayload>> {
    const WorldService* world_service =
        context.context.GetServiceOrNull<WorldService>();
    if (world_service == nullptr) {
      return absl::NotFoundError(absl::Substitute(
          "HalArmPart '$0' is attempting to fetch the payload from the "
          "world, but there's no world service available",
          context.part_name));
    }
    std::shared_ptr<const world::ObjectWorldClient> world =
        world_service->GetObjectWorldClient();
    INTR_ASSIGN_OR_RETURN(world::KinematicObject robot,
                          world->GetKinematicObject(
                              WorldObjectName(context.hardware_resource_name)));
    return robot.GetMountedPayload();
  };

  auto maybe_payload = fetch_payload_from_world();
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<RigidBodyInterface> dynamics_interface,
      GetDynamicsInterface(context.part_name, context.hardware_resource_name,
                           proto_config, dynamics_service_ptr));
  if (dynamics_interface != nullptr) {
    if (maybe_payload.ok() && maybe_payload->has_value()) {
      auto manipulator_kinematics_interface =
          part->GetFeatureInterfaces().GetInterface<ManipulatorKinematics>();
      if (manipulator_kinematics_interface == nullptr) {
        return absl::FailedPreconditionError(
            "ManipulatorKinematics interface is not available, cannot set end "
            "effector dynamics parameters.");
      }
      INTR_ASSIGN_OR_RETURN(
          const kinematics::ElementId non_branching_tip_id,
          manipulator_kinematics_interface->GetKinematicsModel()
              .FindNonBranchingKinematicChainTip());
      INTR_RETURN_IF_ERROR(dynamics_interface->SetEndEffectorDynamicsParameters(
          maybe_payload.value()->tip_t_cog(), maybe_payload.value()->mass(),
          maybe_payload.value()->inertia(), non_branching_tip_id));
    }
    CHECK_OK(part->RegisterDynamicsInterface(
        std::make_unique<DynamicsImpl>(std::move(dynamics_interface))));
  }

  if (has_payload_command) {
    std::optional<RobotPayload> payload;
    if (maybe_payload.ok()) {
      payload = *maybe_payload;
    } else {
      LOG(WARNING) << "Failed to read payload from world: "
                   << maybe_payload.status().message();
    }
    if (!payload.has_value()) {
      LOG(INFO)
          << "No payload set in world. ICON will not set the payload and "
             "relies on the payload configured on the robot controller. The "
             "payload can be updated using the set_payload skill.";
    } else {
      LOG(INFO) << "Initializing full_payload from world: " << *payload;
    }

    INTR_ASSIGN_OR_RETURN(
        part->part_property_ids_.full_payload,
        PayloadProperty::Create(kFullPayloadPropertyName, payload,
                                context.property_registry));

    INTR_ASSIGN_OR_RETURN(
        auto payload_command_feature_interface,
        (MapMutableHardwareInterfaceToFeatureInterface<
            intrinsic_fbs::PayloadCommand, PayloadCommandFeature>(
            context.context, proto_config.payload_command(),
            part->part_property_ids_.full_payload, payload)),
        _ << "for payload_command");
    INTR_RETURN_IF_ERROR(
        part->RegisterInterface(std::move(payload_command_feature_interface)));
    INTR_RETURN_IF_ERROR(
        part->AddHardwareModule(proto_config.payload_command().module_name()));

    if (has_payload_state) {
      INTR_ASSIGN_OR_RETURN(
          auto payload_state_feature_interface,
          (MapMutableHardwareInterfaceToFeatureInterface<
              intrinsic_fbs::PayloadState, PayloadStateFeature>(
              context.context, proto_config.payload_state(),
              part->part_property_ids_.full_payload)),
          _ << "for payload_state");
      INTR_RETURN_IF_ERROR(
          part->RegisterInterface(std::move(payload_state_feature_interface)));
      INTR_RETURN_IF_ERROR(
          part->AddHardwareModule(proto_config.payload_state().module_name()));
      LOG(INFO) << "Registered PayloadStateFeature.";
    }
  } else {
    if (has_payload_state) {
      return absl::InvalidArgumentError(
          "Cannot register a `payload_state` without a `payload_command`.");
    }
  }

  INTR_ASSIGN_OR_RETURN(
      auto generic_config,
      ExtractGenericConfigWithSafetyLimits(*part, context.part_name));
  return PartPtrAndGenericConfig{.part_ptr = std::move(part),
                                 .config = std::move(generic_config)};
}

}  // namespace intrinsic::icon
