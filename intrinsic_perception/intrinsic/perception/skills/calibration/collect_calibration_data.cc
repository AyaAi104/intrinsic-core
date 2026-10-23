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

#include "intrinsic/perception/skills/calibration/collect_calibration_data.h"

#include <memory>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "google/protobuf/message.h"
#include "grpcpp/client_context.h"
#include "intrinsic/assets/dependencies/utils.h"
#include "intrinsic/assets/proto/v1/resolved_dependency.pb.h"
#include "intrinsic/connect/cc/grpc/channel.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/cc_client/client.h"
#include "intrinsic/icon/equipment/resolved_dependency_utils.h"
#include "intrinsic/icon/proto/cart_space_conversion.h"
#include "intrinsic/icon/proto/part_status.pb.h"
#include "intrinsic/icon/skills/util/position_part_util.h"
#include "intrinsic/icon/utils/arm_utils.h"
#include "intrinsic/logging/proto/context.pb.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/motion_planning/motion_planner_client.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/service/motion_planner_service_asset_utils.h"
#include "intrinsic/perception/proto/v1/calibration_service.grpc.pb.h"
#include "intrinsic/perception/proto/v1/calibration_service.pb.h"
#include "intrinsic/perception/skills/calibration/calibration_robot_motion_utils.h"
#include "intrinsic/perception/skills/calibration/collect_calibration_data.pb.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/objects/frame.h"
#include "intrinsic/world/objects/kinematic_object.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/world_object.h"
#include "intrinsic/world/proto/collision_settings.pb.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"
#include "intrinsic/world/util/object_reference_utils.h"

namespace intrinsic {
namespace skills {

struct CollectCalibrationData::InputParameters {
  world::KinematicObject robot;
  world::Frame flange;
  world::WorldObject calibration_object;
  intrinsic_proto::world::ObjectReference calibration_object_reference;
  bool skip_return_to_base_between_waypoints;
  CalibrationRobotMotionConfig motion_config;
};

std::unique_ptr<SkillInterface> CollectCalibrationData::CreateSkill() {
  return std::make_unique<CollectCalibrationData>();
}

namespace {

constexpr absl::Duration kCaptureDataTimeout = absl::Seconds(600);

absl::Status ValidateParams(
    const intrinsic_proto::skills::CollectCalibrationDataParams& params) {
  if (IsEmptyObjectReference(params.calibration_object())) {
    return absl::InvalidArgumentError("calibration_object cannot be empty.");
  }

  // Parameters for automatic calibration only need to be checked if no
  // waypoints are given.
  if (params.waypoints_size() == 0) {
    return absl::InvalidArgumentError("No waypoints provided.");
  }

  return absl::OkStatus();
}

}  // namespace

absl::Status CollectCalibrationData::CollectDataWithCalibrationService(
    const InputParameters& input_params, ExecuteContext& context,
    std::unique_ptr<
        intrinsic_proto::perception::v1::CalibrationService::StubInterface>
        calibration_service_stub,
    const ::google::protobuf::RepeatedPtrField<
        intrinsic_proto::motion_planning::v1::GeometricConstraint>& waypoints,
    const intrinsic_proto::motion_planning::v1::GeometricConstraint&
        waypoint_initial,
    const intrinsic_proto::world::TransformNodeReference& tool,
    const intrinsic_proto::world::TransformNodeReference& frame) {
  if (calibration_service_stub == nullptr) {
    return absl::FailedPreconditionError(
        "Data collection failed because calibration service stub has not been "
        "initialized.");
  }
  world::ObjectWorldClient& world = context.object_world();
  INTR_ASSIGN_OR_RETURN(const intrinsic::world::WorldObject root,
                        world.GetRootObject());
  int collected_waypoints_count = 0;
  for (int i = 0; i < waypoints.size(); ++i) {
    const auto& waypoint = waypoints[i];
    LOG(INFO) << "Moving to waypoint " << i + 1 << "/" << waypoints.size()
              << "\n"
              << waypoint.DebugString();
    absl::Status status = MoveToWaypoint(waypoint, tool, frame,
                                         input_params.motion_config, context);
    if (!status.ok()) {
      if (absl::IsCancelled(status)) {
        return status;
      }
      LOG(WARNING) << "Could not reach requested waypoint. Skipping."
                   << status.message();
      continue;
    }

    INTR_ASSIGN_OR_RETURN(
        Pose3d base_t_flange,
        world.GetTransform(input_params.robot, input_params.flange));

    // We need a new context for each gRPC call.
    auto calibration_service_context = std::make_unique<grpc::ClientContext>();
    calibration_service_context->set_deadline(
        absl::ToChronoTime(absl::Now() + kCaptureDataTimeout));
    intrinsic_proto::perception::v1::CaptureDataRequest capture_data_request;
    intrinsic_proto::perception::v1::CaptureDataResponse capture_data_response;
    *capture_data_request.mutable_base_t_flange() = ToProto(base_t_flange);
    INTR_RETURN_IF_ERROR(ToAbslStatus(calibration_service_stub->CaptureData(
        calibration_service_context.get(), capture_data_request,
        &capture_data_response)));

    collected_waypoints_count++;

    // TODO(b/429345244): Check and react to the returned CaptureDataStatus?

    // After each waypoint, return the robot to its initial position.
    if (!input_params.skip_return_to_base_between_waypoints) {
      INTR_RETURN_IF_ERROR(MoveToWaypoint(waypoint_initial, tool, frame,
                                          input_params.motion_config, context));
    }
  }
  LOG(INFO) << "Collected " << collected_waypoints_count << " of "
            << waypoints.size() << " waypoints.";
  return absl::OkStatus();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
CollectCalibrationData::Execute(const ExecuteRequest& request,
                                ExecuteContext& context) {
  INTR_ASSIGN_OR_RETURN(
      auto params,
      request.params<intrinsic_proto::skills::CollectCalibrationDataParams>());
  INTR_RETURN_IF_ERROR(ValidateParams(params));
  world::ObjectWorldClient& world = context.object_world();

  // Backup initial robot joint configuration so robot can return to this
  // configuration after calibration.
  eigenmath::VectorXd robot_start_joint_configuration;
  // We need to talk to ICON directly because the world could be stale.
  INTR_ASSIGN_OR_RETURN(icon::IconResolvedDependency icon_resolved_dependency,
                        icon::ConnectToIconResolvedDependency(params.robot()));
  INTR_ASSIGN_OR_RETURN(
      icon::PositionPartWithObject part_with_object,
      icon_resolved_dependency.GetPositionPartAndKinematicObject(
          world, params.arm_part()));
  const world::KinematicObject& robot = part_with_object.kinematic_object;
  INTR_ASSIGN_OR_RETURN(const world::Frame flange,
                        robot.GetSingleIsoFlangeFrame());

  icon::Client icon_client(icon_resolved_dependency.channel);
  INTR_ASSIGN_OR_RETURN(
      intrinsic_proto::icon::PartStatus part_status,
      icon_client.GetSinglePartStatus(part_with_object.part_name));
  // Update the world with robot's joint configuration so are based on the
  // current robot configuration.

  robot_start_joint_configuration =
      icon::GetCommandedOrSensedJointPosition(part_status);
  INTR_RETURN_IF_ERROR(
      world.UpdateJointPositions(robot, robot_start_joint_configuration));
  // Store initial robot position so we can return to it.
  auto waypoint_initial =
      intrinsic_proto::motion_planning::v1::GeometricConstraint();
  VectorXdToRepeatedDouble(
      robot_start_joint_configuration,
      waypoint_initial.mutable_joint_position()->mutable_joints());
  INTR_ASSIGN_OR_RETURN(const world::WorldObject calibration_object,
                        world.GetObject(params.calibration_object()));

  constexpr absl::string_view kCalibrationInterface =
      "grpc://intrinsic_proto.perception.v1.CalibrationService";

  // Connect to the Calibration service.
  INTR_ASSIGN_OR_RETURN(std::shared_ptr<grpc::Channel> calibration_channel,
                        intrinsic::assets::dependencies::Connect(
                            params.calibration_service(), kCalibrationInterface,
                            connect::UnlimitedMessageSizeGrpcChannelArgs()));
  std::unique_ptr<
      intrinsic_proto::perception::v1::CalibrationService::StubInterface>
      calibration_service_stub =
          intrinsic_proto::perception::v1::CalibrationService::NewStub(
              calibration_channel);
  if (calibration_service_stub == nullptr) {
    return absl::FailedPreconditionError(
        "Calibration service stub could not be created.");
  }

  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<motion_planning::MotionPlannerClient>
          motion_planner_service_asset_client,
      GetMotionPlannerServiceAssetClient(world.GetWorldID(),
                                         params.motion_planner_service()));
  // TODO(b/524634328): Remove the Fallback Logic from the Skills.
  motion_planning::MotionPlannerClient& motion_planner =
      (motion_planner_service_asset_client != nullptr)
          ? *motion_planner_service_asset_client
          : context.motion_planner();

  // Should not be here if simulation level is kinematics sim but
  // use_camera_rendering_in_draft_mode_sim is false.
  CollectCalibrationData::InputParameters input_params{
      .robot = robot,
      .flange = flange,
      .calibration_object = calibration_object,
      .skip_return_to_base_between_waypoints =
          params.skip_return_to_base_between_waypoints(),
      .motion_config =
          CalibrationRobotMotionConfig{
              .robot = robot,
              .disable_collision_checking = params.disable_collision_checking(),
              .motion_type = params.motion_type(),
              .arm_part_info =
                  ArmPartInformation{
                      .name = part_with_object.part_name,
                      .object = robot.ObjectReference(),
                  },
              .icon_channel = icon_resolved_dependency.channel,
              .motion_planner = motion_planner,
          },
  };

  context.canceller().Ready();

  const auto tool = flange.TransformNodeReference();
  const auto frame = robot.TransformNodeReference();

  INTR_RETURN_IF_ERROR(CollectDataWithCalibrationService(
                           input_params, context,
                           std::move(calibration_service_stub),
                           params.waypoints(), waypoint_initial, tool, frame))
      .LogError();

  // After calibration, return the robot to its initial position.
  INTR_RETURN_IF_ERROR(MoveToWaypoint(waypoint_initial, tool, frame,
                                      input_params.motion_config, context));
  auto return_value =
      std::make_unique<intrinsic_proto::skills::CollectCalibrationDataResult>();
  return return_value;
}

absl::StatusOr<std::unique_ptr<::google::protobuf::Message>>
CollectCalibrationData::Preview(const PreviewRequest& request,
                                PreviewContext& context) {
  return std::make_unique<
      intrinsic_proto::skills::CollectCalibrationDataResult>();
}

absl::StatusOr<intrinsic_proto::skills::Footprint>
CollectCalibrationData::GetFootprint(const GetFootprintRequest& request,
                                     GetFootprintContext& context) const {
  intrinsic_proto::skills::Footprint result;
  // We lock the universe and thus prevent parallel execution for the execution
  // of calibration.
  result.set_lock_the_universe(true);
  return result;
}

}  // namespace skills
}  // namespace intrinsic
