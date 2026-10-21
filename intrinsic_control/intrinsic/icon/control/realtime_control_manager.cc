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

#include "intrinsic/icon/control/realtime_control_manager.h"

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/cleanup/cleanup.h"
#include "absl/container/fixed_array.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "grpc/grpc.h"
#include "grpcpp/channel.h"
#include "grpcpp/server_builder.h"
#include "grpcpp/support/channel_arguments.h"
#include "intrinsic/hardware/gpio/icon_gpio_service.h"
#include "intrinsic/hardware/gpio/icon_gpio_service_config.pb.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/topic_names.h"
#include "intrinsic/icon/control/behavior_override.h"
#include "intrinsic/icon/control/context.h"
#include "intrinsic/icon/control/logging_mode.h"
#include "intrinsic/icon/control/parts/hal/v1/hal_part_config.pb.h"
#include "intrinsic/icon/control/parts/part_property_registry.h"
#include "intrinsic/icon/control/parts/proto/v1/realtime_part_config.pb.h"
#include "intrinsic/icon/control/parts/realtime_log_context.h"
#include "intrinsic/icon/control/parts/realtime_part.h"
#include "intrinsic/icon/control/parts/realtime_part_status.h"
#include "intrinsic/icon/control/plugin_action.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/realtime_function_bridge.h"
#include "intrinsic/icon/control/realtime_operational_state_bridge.h"
#include "intrinsic/icon/control/realtime_operational_status.h"
#include "intrinsic/icon/control/realtime_part_manager.h"
#include "intrinsic/icon/control/realtime_session_channels.h"
#include "intrinsic/icon/control/realtime_state_manager_interface.h"
#include "intrinsic/icon/control/robot_connection_rtcl_controller.h"
#include "intrinsic/icon/control/rtcl_action_instance.h"
#include "intrinsic/icon/control/rtcl_controller_bridge.h"
#include "intrinsic/icon/control/rtcl_initialization.h"
#include "intrinsic/icon/control/rtcl_safety_message_handler.h"
#include "intrinsic/icon/control/rtcl_session_manager.h"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/control/services/world_service.h"
#include "intrinsic/icon/hal/default_hardware_interfaces.h"  // IWYU pragma: keep
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/part_status.pb.h"
#include "intrinsic/icon/proto/safety_status_conversion.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/config/realtime_control_config.pb.h"
#include "intrinsic/icon/server/custom_action_plugin_loader.h"
#include "intrinsic/icon/server/jogging_service.h"
#include "intrinsic/icon/server/operational_state_interface.h"
#include "intrinsic/icon/server/runtime_settings_database.h"
#include "intrinsic/icon/server/service.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/logging/data_logger_client.h"
#include "intrinsic/logging/log_item_builder.h"
#include "intrinsic/logging/proto/log_item.pb.h"
#include "intrinsic/logging/utils/logger_labels.h"
#include "intrinsic/platform/common/buffers/realtime_write_queue.h"
#include "intrinsic/platform/pubsub/publisher.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/grpc/channel.h"
#include "intrinsic/util/grpc/limits.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/rt_thread.h"
#include "intrinsic/util/thread/rt_trace.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"
#include "intrinsic/world/objects/object_world_client.h"
#include "ortools/base/path.h"

namespace intrinsic::icon {
namespace {

using ::intrinsic_fbs::EnumNameModeOfSafeOperation;
using ::intrinsic_fbs::EnumNameRequestedBehavior;
using ::intrinsic_proto::data_logger::LogItem;
using ::intrinsic_proto::icon::SafetyMessageHandlerConfig;

absl::Status CheckForDuplicateBlockNameAndInsert(
    absl::string_view block_name,
    absl::flat_hash_set<std::string>& existing_block_names) {
  if (existing_block_names.contains(block_name)) {
    return absl::InvalidArgumentError(
        absl::StrCat("Duplicate block name: ", block_name));
  }
  existing_block_names.insert(std::string(block_name));
  return absl::OkStatus();
}

absl::Status ValidateAdioForDuplicateBlockNames(
    const RtclPartsAndConfigs& parts_and_configs) {
  absl::flat_hash_set<std::string> existing_adio_block_names;
  for (const auto& part_config : parts_and_configs.part_configs) {
    if (!part_config.generic_config().has_adio_config()) {
      continue;
    }
    const auto& adio_config = part_config.generic_config().adio_config();
    for (const auto& block : adio_config.analog_input_blocks()) {
      INTR_RETURN_IF_ERROR(CheckForDuplicateBlockNameAndInsert(
          block.first, existing_adio_block_names));
    };
    for (const auto& block : adio_config.digital_input_blocks()) {
      INTR_RETURN_IF_ERROR(CheckForDuplicateBlockNameAndInsert(
          block.first, existing_adio_block_names));
    };
    for (const auto& block : adio_config.analog_output_blocks()) {
      INTR_RETURN_IF_ERROR(CheckForDuplicateBlockNameAndInsert(
          block.first, existing_adio_block_names));
    };
    for (const auto& block : adio_config.digital_output_blocks()) {
      INTR_RETURN_IF_ERROR(CheckForDuplicateBlockNameAndInsert(
          block.first, existing_adio_block_names));
    };
  }
  return absl::OkStatus();
}

// Selects the part_context with the highest icon_action_id to be used as
// context for the whole PartStatus.
// TODO(b/273733857): Remove this function and publish all part_contexts
// directly.
RealtimeLogContext SelectContextForPartStatus(
    absl::Span<const RealtimeLogContext> contexts) {
  RealtimeLogContext context{};
  for (size_t i = 0; i < contexts.size(); i++) {
    const RealtimeLogContext& part_context = contexts.at(i);
    if (part_context == RealtimeLogContext{}) {
      continue;
    }
    if (part_context == context) {
      continue;
    }
    if (context == RealtimeLogContext{}) {
      context = part_context;
      continue;
    }
    LOG_EVERY_N_SEC(WARNING, 1.0)
        << "Contexts are not equal. The last part context is "
           "used. This can happen if parallel sessions exist.";
    if (part_context.icon_action_id.has_value() && part_context.skill_id > 0 &&
        part_context.icon_action_id > context.icon_action_id) {
      context = part_context;
    }
  }
  return context;
}

void PublishPartStatusesSaveRuntimeSettings(
    std::string topic_full_rate, std::string topic_decimated,
    const double control_frequency_hz,
    std::vector<std::string> part_names_in_index_order,
    std::atomic_bool* shutdown,
    RealtimeWriteQueue<PublishOutput>::NonRtReader& part_status_reader,
    OperationalStateInterface& operational_state, PubSub& pub_sub,
    InitializedAsyncBuffer<LoggingMode>& logging_mode_buffer,
    std::string runtime_settings_file,
    std::atomic<double>& speed_override_buffer) {
  INTRINSIC_ASSERT_NON_REALTIME();
  const auto decimation_factor =
      CalculateDecimationFactor(control_frequency_hz, kThrottledStatusRate);
  if (!decimation_factor.ok()) {
    LOG(ERROR) << "Failed to calculate decimation factor: "
               << decimation_factor.status()
               << ", aborting PublishPartStatus().";
    return;
  }
  PublishOutput output;

  absl::StatusOr<Publisher> maybe_pub_decimated, maybe_pub_full_rate;
  if (topic_decimated.empty() || topic_full_rate.empty()) {
    LOG(ERROR) << "Got empty topic. Will not publish the RobotStatus.";
  } else if (topic_decimated == topic_full_rate) {
    LOG(ERROR) << "Same topic name provided for full_rate and decimated "
                  "status. Will not publish Robot Status messages.";
  } else {
    LOG(INFO) << "Publishing part status to topic: " << topic_decimated
              << " and full-rate status to topic: " << topic_full_rate;
    maybe_pub_decimated =
        pub_sub.CreatePublisher(topic_decimated, TopicConfig());
    maybe_pub_full_rate =
        pub_sub.CreatePublisher(topic_full_rate, TopicConfig());
  }
  absl::Time last_update = absl::InfiniteFuture();
  std::optional<RuntimeSettings> previous_runtime_settings;
  while (!shutdown->load()) {
    INTRINSIC_TRACE_SCOPED("PublishPartStatusesSaveRuntimeSettings");
    {
      RuntimeSettings runtime_settings{.speed_override = speed_override_buffer};
      if (!previous_runtime_settings ||
          *previous_runtime_settings != runtime_settings) {
        previous_runtime_settings = runtime_settings;
        auto status =
            SaveRuntimeSettings(runtime_settings_file, runtime_settings);
        if (!status.ok()) {
          LOG_EVERY_N_SEC(INFO, 60)
              << "Failed to save runtime settings to file: "
              << runtime_settings_file << ": " << status
              << " (Is a volume mounted in the resource manifest?)";
        }
      }
    }
    if (!maybe_pub_decimated.ok() || !maybe_pub_full_rate.ok()) {
      absl::SleepFor(absl::Seconds(1));
      continue;
    }

    intrinsic::ReadResult part_status_read_result =
        part_status_reader.ReadWithTimeout(output,
                                           absl::Now() + absl::Seconds(1));

    if (part_status_read_result == intrinsic::ReadResult::kClosed) {
      LOG(ERROR) << "Part status queue closed unexpectedly.";
      return;
    }
    if (part_status_read_result == intrinsic::ReadResult::kDeadlineExceeded) {
      if (last_update == absl::InfiniteFuture()) {
        LOG(INFO)
            << "Timed out waiting for first message from Part status queue, "
               "will retry.";
        continue;
      }
      LOG(ERROR) << "Timed out reading from Part status queue, will retry. "
                    "Timeslicers realtime thread may be blocked. Duration "
                    "since last message: "
                 << absl::Now() - last_update;
      continue;
    }
    if (part_status_read_result != intrinsic::ReadResult::kConsumed) {
      LOG(ERROR) << "Got unexpected result while reading from Part status "
                    "queue, aborting PublishPartStatus().";
      return;
    }
    last_update = absl::Now();
    intrinsic_proto::icon::RobotStatus robot_status;
    *robot_status.mutable_safety_status() =
        ToProto(output.robot_status.safety_status);
    {
      auto maybe_op_status = operational_state.GetStatus();
      if (maybe_op_status.ok()) {
        *robot_status.mutable_operational_status() = ToProto(*maybe_op_status);
      }
    }
    robot_status.set_speed_override(previous_runtime_settings->speed_override);

    for (size_t i = 0; i < output.robot_status.part_statuses.size(); ++i) {
      // Default part_name to index in case there is no mapping.
      std::string part_name = std::to_string(i);
      if (i < part_names_in_index_order.size()) {
        part_name = part_names_in_index_order.at(i);
      } else {
        LOG_EVERY_N_SEC(ERROR, 1)
            << "Failed to lookup the name for the part with index[" << i
            << "]. Using the index as name.";
      }
      robot_status.mutable_status_map()->insert(
          {part_name, ToProto(output.robot_status.part_statuses.at(i))});
    }

    if (output.robot_status.part_statuses.size() !=
        output.part_contexts.size()) {
      LOG_EVERY_N_SEC(ERROR, 1)
          << "Number of part statuses is "
          << output.robot_status.part_statuses.size()
          << ", number of contexts is " << output.part_contexts.size()
          << ". Should be equal.";
    }

    robot_status.set_wall_clock_timestamp_ns(
        absl::ToUnixNanos(output.wall_time));
    robot_status.set_timestamp_ns(
        absl::ToInt64Nanoseconds(output.timestamp_control));

    RealtimeLogContext context =
        SelectContextForPartStatus(output.part_contexts);

    LogItem log_item = data_logger::Builder::From(robot_status)
                           .WithEventSource(topic_full_rate)
                           .WithContext(ToProto(context))
                           .Item();

    if (absl::Status status = AddSkillLogIdLabel(*log_item.mutable_context());
        !status.ok()) {
      LOG_EVERY_N_SEC(ERROR, 1)
          << "Failed to add skill id label to context: " << status;
    }

    if (absl::Status status =
            maybe_pub_full_rate->Publish(log_item, output.wall_time);
        !status.ok()) {
      LOG_EVERY_N_SEC(ERROR, 1)
          << "Failed to publish full-rate status on topic [" << topic_full_rate
          << "]. Dropping.";
    }

    const bool publish_decimated =
        IsThrottledLogCycle(output.cycle, *decimation_factor);

    const LoggingMode logging_mode =
        logging_mode_buffer.GetCurrentValue(absl::UnixEpoch())
            .value_or(LoggingMode::kThrottled);

    if (log_item.context().has_icon_session_id() &&
        (logging_mode == LoggingMode::kFullRate || publish_decimated)) {
      data_logger::LogAsync(log_item, [](absl::Status status) {
        if (!status.ok()) {
          LOG_EVERY_N_SEC(WARNING, 10)
              << "Failed to log robot-status: " << status;
        }
      });
    }

    if (publish_decimated) {
      log_item.mutable_metadata()->set_event_source(topic_decimated);
      if (absl::Status status =
              maybe_pub_decimated->Publish(log_item, output.wall_time);
          !status.ok()) {
        LOG_EVERY_N_SEC(WARNING, 1)
            << "Failed to publish decimated status on topic ["
            << topic_decimated << "]. Dropping.";
      }
    }
  }
}

}  // namespace

// static
absl::StatusOr<std::unique_ptr<RealtimeControlManager>>
RealtimeControlManager::Create(
    const intrinsic_proto::icon::RealtimeControlConfig& config,
    const ::intrinsic_proto::icon::v1::ServerConfig& server_config,
    const Context& context,
    RealtimeStateManagerInterface& realtime_state_manager,
    LoggingMode default_logging_mode) {
  for (const auto& [part_name, part_config] : config.parts_by_name()) {
    if (part_config.part_type_name().empty()) {
      return absl::InvalidArgumentError(
          absl::StrCat("Config for Part '", part_name,
                       "' is missing required field part_type_name"));
    }
    if (part_config.safety_action_type_name().empty()) {
      return absl::InvalidArgumentError(
          absl::StrCat("Config for Part '", part_name, "' of type '",
                       part_config.part_type_name(),
                       "' is missing required field safety_action_type_name"));
    }
  }

  INTR_ASSIGN_OR_RETURN(
      RtclPartsAndConfigs parts_and_configs,
      InitializePartsFromProto(
          server_config,
          absl::flat_hash_map<std::string,
                              intrinsic_proto::v1::RealtimePartConfig>{
              config.parts_by_name().begin(), config.parts_by_name().end()},
          context));

  INTR_RETURN_IF_ERROR(ValidateAdioForDuplicateBlockNames(parts_and_configs));

  SafetyMessageHandlerConfig message_handler_config;
  if (config.has_safety_hardware_interface()) {
    LOG(INFO) << "RealtimeControlManager created with legacy "
                 "`safety_hardware_interface` config";
    message_handler_config.add_safety_hardware_interfaces()->CopyFrom(
        config.safety_hardware_interface());
  } else if (config.has_safety_message_handler_config()) {
    message_handler_config = config.safety_message_handler_config();
  }

  return Create(std::move(parts_and_configs), context, realtime_state_manager,
                server_config.frequency_hz(), default_logging_mode,
                message_handler_config);
}

// static
absl::StatusOr<std::unique_ptr<RealtimeControlManager>>
RealtimeControlManager::Create(
    RtclPartsAndConfigs parts_and_configs, const Context& context,
    RealtimeStateManagerInterface& realtime_state_manager,
    double control_frequency_hz, LoggingMode default_logging_mode,
    SafetyMessageHandlerConfig safety_message_handler_config) {
  std::unique_ptr<RealtimeControlManager> realtime_control_manager =
      absl::WrapUnique(new RealtimeControlManager(
          std::move(parts_and_configs), realtime_state_manager,
          default_logging_mode, safety_message_handler_config));
  INTR_RETURN_IF_ERROR(
      realtime_control_manager->Initialize(context, control_frequency_hz));

  return std::move(realtime_control_manager);
}

// Constructs from RtclPartsAndConfigs struct and safety module information.
RealtimeControlManager::RealtimeControlManager(
    RtclPartsAndConfigs parts_and_configs,
    RealtimeStateManagerInterface& realtime_state_manager,
    LoggingMode default_logging_mode,
    intrinsic_proto::icon::SafetyMessageHandlerConfig
        safety_message_handler_config)
    : realtime_state_manager_(realtime_state_manager),
      operational_state_(operational_state_command_bridge_,
                         operational_cell_control_status_buffer_),
      logging_mode_buffer_(default_logging_mode),
      parts_and_configs_(std::move(parts_and_configs)),
      session_managers_(parts_and_configs_.part_configs.size() +
                        kMaxNumberOfMonitoringSessions),
      safety_message_handler_config_(safety_message_handler_config) {
  // Ensure the logging buffer starts with the correct default mode.
  *logging_mode_buffer_.GetFreeBuffer() = default_logging_mode;
  logging_mode_buffer_.CommitFreeBuffer();
}

RealtimeControlManager::~RealtimeControlManager() {
  Shutdown(AbortedError("Shutting down."));
  // Not joinable if controller not initialize()d
  if (publishing_thread_.joinable()) {
    publishing_thread_.join();
  }
}

void RealtimeControlManager::Shutdown(RealtimeStatus status) {
  shutdown_publishing_thread_ = true;
  operational_state_command_bridge_.Close();

  auto abort_operational_state_command =
      [status](RealtimeFunctionBridge<RealtimeStatus(
                   OperationalStateCommand)>::RealtimeFunctionHandle& handle) {
        while (!handle.TryLockAndExecute(
            [&status](RealtimeStatus& return_status) -> RealtimeFunctionState {
              return_status = status;
              return RealtimeFunctionState::kDone;
            })) {
          absl::SleepFor(absl::Milliseconds(10));
        }
      };
  // Finish ongoing operational state command, if any.
  if (current_operational_state_command_handle_.has_value()) {
    abort_operational_state_command(
        current_operational_state_command_handle_.value().handle);
    current_operational_state_command_handle_.reset();
  }
  // Finish all pending operational state commands.
  auto new_handle = operational_state_command_bridge_.NextRealtimeHandle();
  while (new_handle.has_value()) {
    abort_operational_state_command(*new_handle);
    new_handle = operational_state_command_bridge_.NextRealtimeHandle();
  }
  if (part_manager_ != nullptr) {
    part_manager_->CurrentRobotStatusBuffer().Shutdown();
  }
  SetSessionManagerErrorCleanupNotification(status);
  LOG(INFO) << "RealtimeControlManager has shut down.";
}

std::shared_ptr<grpc::Channel> RealtimeControlManager::InProcChannel(
    const grpc::ChannelArguments& channel_args) {
  INTRINSIC_ASSERT_NON_REALTIME();
  if (!icon_in_process_server_) {
    return nullptr;
  }
  return icon_in_process_server_->InProcessChannel(channel_args);
}

absl::StatusOr<std::unique_ptr<intrinsic_proto::gpio::v1::GPIOService::Service>>
StartGpioService(
    std::string_view instance_name, std::shared_ptr<intrinsic::Channel> channel,
    std::span<const intrinsic_proto::icon::v1::PartConfig> part_configs) {
  // Query all parts that have an ADIO interface.
  std::vector<std::string> adio_names;

  for (const auto& part_config : part_configs) {
    if (absl::c_find(part_config.feature_interfaces(),
                     intrinsic_proto::icon::v1::FeatureInterfaceTypes::
                         FEATURE_INTERFACE_ADIO) !=
        part_config.feature_interfaces().end()) {
      adio_names.push_back(part_config.name());
    }
  }

  intrinsic_proto::gpio::IconGpioServiceConfig gpio_config;
  gpio_config.mutable_parts()->Add(adio_names.begin(), adio_names.end());
  gpio_config.set_equipment_instance_name(instance_name);
  gpio_config.set_robot(instance_name);

  return intrinsic::MakeIconGPIOService(gpio_config, channel);
}

absl::StatusOr<
    std::unique_ptr<::intrinsic_proto::icon::v1::JoggingService::Service>>
StartJoggingService(const Context& context,
                    std::shared_ptr<intrinsic::Channel> channel) {
  auto* icon_world_service = context.GetServiceOrNull<WorldService>();
  std::shared_ptr<const world::ObjectWorldClient> object_world_client =
      icon_world_service != nullptr ? icon_world_service->GetObjectWorldClient()
                                    : nullptr;

  return IconJoggingService::Create(channel, std::move(object_world_client));
}

absl::Status RealtimeControlManager::StartGpioAndJoggingServers(
    const Context& context) {
  // Start an in-process ICON API gRPC server for the GPIO and jogging service
  // to connect to.
  std::unique_ptr<grpc::Server> icon_in_process_server;
  {
    grpc::ServerBuilder builder;
    builder.AddChannelArgument(GRPC_ARG_ALLOW_REUSEPORT, 0);
    builder.AddChannelArgument(GRPC_ARG_MAX_METADATA_SIZE,
                               kGrpcRecommendedMaxMetadataSoftLimit);
    builder.AddChannelArgument(GRPC_ARG_ABSOLUTE_MAX_METADATA_SIZE,
                               kGrpcRecommendedMaxMetadataHardLimit);
    builder.RegisterService(icon_service_.get());
    icon_in_process_server = builder.BuildAndStart();
    if (icon_in_process_server == nullptr) {
      return absl::InternalError(
          "Failed to build in-process ICON API server for extra services "
          "(GPIO, Jogging).");
    }
  }

  // Create a connection to the in process icon instance.
  auto channel = std::make_shared<intrinsic::Channel>(
      icon_in_process_server->InProcessChannel(grpc::ChannelArguments()));

  INTR_ASSIGN_OR_RETURN(
      auto gpio_service,
      StartGpioService(server_config_.name(), channel,
                       robot_connection_->GetPartCollection().GetPartConfigs()),
      _.SetPrepend() << "Failed to start GPIO service: ");
  LOG(INFO) << "PUBLIC: Started GPIO service.";

  INTR_ASSIGN_OR_RETURN(
      auto jogging_service, StartJoggingService(context, channel),
      _.SetPrepend() << "PUBLIC: Failed to start jogging service: ");
  LOG(INFO) << "PUBLIC: Started jogging service.";

  // Using temporary variables for the services and moving them here, instead of
  // assigning when they are created, ensures that they are deleted when there
  // is an error.
  icon_in_process_server_ = std::move(icon_in_process_server);
  gpio_service_ = std::move(gpio_service);
  jogging_service_ = std::move(jogging_service);

  return absl::OkStatus();
}

void RealtimeControlManager::SetSessionManagerErrorCleanupNotification(
    RealtimeStatus status) {
  // End all Sessions. This signals the error to clients as well.
  for (RtclSessionManager& session_manager : session_managers_) {
    session_manager.SetErrorCleanupNotify(status);
  }
}

RealtimeStatusOr<const RtclActionInstance*>
RealtimeControlManager::GetCurrentActionForPartTestOnly(
    absl::string_view part_name) const {
  std::optional<size_t> part_index = std::nullopt;
  for (size_t i = 0; i < part_manager_->GetNumParts(); ++i) {
    if (part_manager_->GetPartName(i) == part_name) {
      part_index = i;
      break;
    }
  }

  if (!part_index.has_value()) {
    return NotFoundError(
        RealtimeStatus::StrCat("Part '", part_name, "' does not exist."));
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(RealtimeOperationalStatus part_status,
                                part_manager_->GetPartState(*part_index));
  if (part_status.state != RealtimeOperationalState::kEnabled) {
    return FailedPreconditionError(
        RealtimeStatus::StrCat("Part '", part_name, "' is not enabled."));
  }

  for (const RtclSessionManager& session_manager : session_managers_) {
    const RtclActionInstance* current_action_ptr =
        session_manager.CurrentActiveActionTestOnly();
    if (current_action_ptr != nullptr &&
        current_action_ptr->visibility_by_slot_id->at(*part_index)) {
      return current_action_ptr;
    }
  }
  // No active Action, return nullptr;
  return nullptr;
}

bool RealtimeControlManager::PartStatusBufferEmptyTestOnly() const {
  return part_manager_->RobotStatusReader().Empty();
};

RealtimeStatusOr<AggregatedRobotStatus>
RealtimeControlManager::PreUpdateSessions(Time cycle_time) {
  // Read and update safety status before the current system state is exported
  // in part_manager. TODO(b/244549017): Maybe split into update and handle.

  // Handle the SafetyStatus before interacting with parts, so that we respond
  // in the same cycle.
  HandleSafetyStatus();
  INTRINSIC_RT_RETURN_IF_ERROR(realtime_state_manager_.UpdateState());
  INTRINSIC_RT_ASSIGN_OR_RETURN(AggregatedRobotStatus robot_status,
                                part_manager_->BeginCycle(cycle_time));
  using RequestState = RealtimeStateManagerInterface::RequestState;
  const RequestState request_state = realtime_state_manager_.GetRequestState();

  // If any part is fatally faulted or init failed, ICON needs to shutdown since
  // the HWM needs to shutdown as well and ICON needs a fresh start to recover.
  // But wait until all HWM are disabled before returning an error.
  if (realtime_state_manager_.CheckAnyInOneOfStates(
          {intrinsic_fbs::StateCode::kInitFailed,
           intrinsic_fbs::StateCode::kFatallyFaulted},
          RealtimeStateManagerInterface::Group::kAllHardware) &&
      !realtime_state_manager_.CheckAnyInOneOfStates(
          {intrinsic_fbs::StateCode::kMotionEnabled,
           intrinsic_fbs::StateCode::kMotionDisabling,
           intrinsic_fbs::StateCode::kMotionEnabling},
          RealtimeStateManagerInterface::Group::kAllHardware)) {
    return FailedPreconditionError(RealtimeStatus::StrCat(
        "Fatal hardware module fault: ",
        realtime_state_manager_.GetFaultReason().value_or(
            RealtimeStateManagerInterface::FaultReasonString("Unknown"))));
  }

  // If no command is active or when the command is partially done, check if
  // any faults are present and propagate by disabling immediately.

  if (request_state == RequestState::kPartiallyDone ||
      request_state == RequestState::kDone) {
    // 1. If a cell control hardware module is faulted or any hardware module is
    // fatally faulted, and any hardware is enabled, disable all hardware.
    bool cell_control_faulted = realtime_state_manager_.CheckAnyInOneOfStates(
        {intrinsic_fbs::StateCode::kInitFailed,
         intrinsic_fbs::StateCode::kFaulted,
         intrinsic_fbs::StateCode::kFatallyFaulted},
        RealtimeStateManagerInterface::Group::kCellControlHardware);
    bool any_fatal_fault = realtime_state_manager_.CheckAnyInOneOfStates(
        {intrinsic_fbs::StateCode::kInitFailed,
         intrinsic_fbs::StateCode::kFatallyFaulted},
        RealtimeStateManagerInterface::Group::kAllHardware);
    if ((cell_control_faulted || any_fatal_fault) &&
        realtime_state_manager_.CheckAnyInOneOfStates(
            {intrinsic_fbs::StateCode::kMotionEnabled},
            RealtimeStateManagerInterface::Group::kAllHardware)) {
      if (cell_control_faulted) {
        INTRINSIC_RT_LOG(INFO)
            << "Disabling all parts because at least one cell "
               "control hardware module is faulted "
               "and at least one is enabled. State summary:\n"
            << realtime_state_manager_.StateSummary();
      } else {
        INTRINSIC_RT_LOG(INFO)
            << "Disabling all parts because at least one hardware module is "
               "fatally faulted and at least one is enabled. State summary:\n"
            << realtime_state_manager_.StateSummary();
      }
      // We need to preempt the ongoing request, by directly calling
      // RequestTransition.
      if (auto status =
              realtime_state_manager_
                  .RequestTransition(RealtimeStateManagerInterface::
                                         TransitionRequest::kDisableMotion)
                  .status();
          !status.ok()) {
        INTRINSIC_RT_LOG(WARNING)
            << "Failed to request preemptive disabling: " << status.ToString();
        return status;
      }
    } else if (
        // 2. If an operational hardware module is faulted, and any operational
        // hardware module is enabled, disable only operational hardware
        // modules.
        realtime_state_manager_.CheckAnyInOneOfStates(
            {intrinsic_fbs::StateCode::kInitFailed,
             intrinsic_fbs::StateCode::kFaulted,
             intrinsic_fbs::StateCode::kFatallyFaulted},
            RealtimeStateManagerInterface::Group::kOperationalHardware) &&
        realtime_state_manager_.CheckAnyInOneOfStates(
            {intrinsic_fbs::StateCode::kMotionEnabled},
            RealtimeStateManagerInterface::Group::kOperationalHardware)) {
      INTRINSIC_RT_LOG(INFO)
          << "Disabling operational hardware because at least one operational "
             "hardware is faulted and at least one is enabled. State summary:\n"
          << realtime_state_manager_.StateSummary();
      // We need to preempt the ongoing request, by directly calling
      // RequestTransition.
      if (auto status =
              realtime_state_manager_
                  .RequestTransition(
                      RealtimeStateManagerInterface::TransitionRequest::
                          kDisableMotionSkipCellControlHardware)
                  .status();
          !status.ok()) {
        INTRINSIC_RT_LOG(WARNING)
            << "Failed to request preemptive disabling: " << status.ToString();
        return status;
      }
    }
  }

  INTRINSIC_RT_RETURN_IF_ERROR(HandleOperationalStatus());
  INTRINSIC_RT_RETURN_IF_ERROR(HandleStartSession());
  return robot_status;
}

RealtimeStatus RealtimeControlManager::Update(Time cycle_time) {
  INTRINSIC_RT_RETURN_IF_ERROR(safety_message_handler_->Update());

  RealtimeStatusOr<AggregatedRobotStatus> robot_status_or_status;
  // TODO(b/288569077): Handle all failures by calling DisableAllParts()
  if (robot_status_or_status = PreUpdateSessions(cycle_time);
      !robot_status_or_status.status().ok()) {
    SetSessionManagerErrorCleanupNotification(robot_status_or_status.status());
    return robot_status_or_status.status();
  }

  if (RealtimeStatus session_update_status =
          UpdateSessions(cycle_time, robot_status_or_status.value());
      !session_update_status.ok()) {
    INTRINSIC_RT_LOG(ERROR)
        << "Session failed: " << session_update_status.ToString();
    INTRINSIC_RT_LOG(ERROR) << "Ending ALL Sessions!";
    // End all Sessions.
    SetSessionManagerErrorCleanupNotification(session_update_status);
    // Disable all parts. `part_manager_` ensures that any active safety actions
    // run to completion before handling this disable command.
    // Processing of requests must be paused until safety actions are done.
    if (auto status =
            realtime_state_manager_
                .RequestTransition(RealtimeStateManagerInterface::
                                       TransitionRequest::kDisableMotion)
                .status();
        !status.ok()) {
      INTRINSIC_RT_LOG(WARNING)
          << "Failed to disable after session failure: " << status.ToString();
      return status;
    }
  }

  // Remember the status of the enable button.
  latest_enable_button_status_ =
      safety_message_handler_->SafetyStatusHandle().enable_button_status();

  return part_manager_->FinishCycle(realtime_state_manager_);
}

RealtimeStatus RealtimeControlManager::UpdateSessions(
    Time cycle_time, const AggregatedRobotStatus& robot_status) {
  // Read speed override value from buffer once, to ensure it is the same for
  // all sessions.
  double speed_override = speed_override_buffer_;
  for (RtclSessionManager& session_manager : session_managers_) {
    INTRINSIC_RT_RETURN_IF_ERROR(
        session_manager.ExecuteCycle(cycle_time, speed_override, robot_status));
  }
  return OkStatus();
}

void RealtimeControlManager::UpdateOperationalStatus(
    bool ignore_active_command) {
  // Update OperationalStatus buffer.
  RealtimeOperationalStateBridge::OperationalCellControlStatus*
      operational_cell_control_status =
          operational_cell_control_status_buffer_.GetFreeBuffer();
  auto commit_operational_cell_control_status = absl::MakeCleanup(
      [this]() { operational_cell_control_status_buffer_.CommitFreeBuffer(); });

  // If we're handling an OperationalStateCommand, use that to determine the
  // OperationalStatus:
  // * If currently enabling, OperationalStatus is still Disabled.
  // * If currently disabling, OperationalStatus is already Disabled, no matter
  // the part statuses.
  // * If currently clearing faults, OperationalStatus is still Faulted.
  if (!ignore_active_command &&
      current_operational_state_command_handle_.has_value()) {
    OperationalStateCommand cmd =
        std::get<0>(*current_operational_state_command_handle_->handle.Args());
    switch (cmd) {
      case OperationalStateCommand::kClearFaults: {
        operational_cell_control_status->operational.state =
            realtime_state_manager_.GetCombinedOperationalState();
        operational_cell_control_status->cell_control.state =
            realtime_state_manager_.GetCellControlState();
        // When clearing faults, operationalstate cannot be better than
        // kFaultedConnected.
        if (operational_cell_control_status->operational.state !=
            RealtimeOperationalState::kFatallyFaulted) {
          operational_cell_control_status->operational.state =
              RealtimeOperationalState::kFaultedConnected;
        }
        std::optional<RealtimeOperationalStatus::FaultReasonString>
            fault_reason = realtime_state_manager_.GetFaultReason();

        if (!fault_reason.has_value()) {
          operational_cell_control_status->operational.fault_reason =
              absl::string_view(
                  "Server is faulted, but no fault reason is currently "
                  "available.");
          operational_cell_control_status->cell_control.fault_reason =
              absl::string_view(
                  "Server is faulted, but no fault reason is currently "
                  "available.");
        } else {
          operational_cell_control_status->operational.fault_reason =
              *fault_reason;
          operational_cell_control_status->cell_control.fault_reason =
              *fault_reason;
        }
        return;
      }
      case OperationalStateCommand::kEnable: {
        operational_cell_control_status->operational.state =
            RealtimeOperationalState::kDisabled;
        operational_cell_control_status->cell_control.state =
            RealtimeOperationalState::kDisabled;
        return;
      }
      case OperationalStateCommand::kDisable: {
        operational_cell_control_status->operational.state =
            RealtimeOperationalState::kDisabled;
        operational_cell_control_status->cell_control.state =
            RealtimeOperationalState::kDisabled;
        return;
      }
      case OperationalStateCommand::kDisableSkipCellControlHardware: {
        operational_cell_control_status->operational.state =
            RealtimeOperationalState::kDisabled;
        operational_cell_control_status->cell_control.state =
            realtime_state_manager_.GetCellControlState();
        return;
      }
      case OperationalStateCommand::kNone:
        // Fall back to checking part states (below) if we don't know the
        // OperationalStateCommand.
        break;
    }
  }
  operational_cell_control_status->operational.state =
      realtime_state_manager_.GetCombinedOperationalState();
  operational_cell_control_status->cell_control.state =
      realtime_state_manager_.GetCellControlState();
  std::optional<RealtimeOperationalStatus::FaultReasonString> fault_reason =
      realtime_state_manager_.GetFaultReason();
  RealtimeStatus session_error =
      AbortedError("Server is faulted: [no fault reason]");
  if (fault_reason.has_value()) {
    session_error = AbortedError(
        RealtimeStatus::StrCat("Server is faulted: ", *fault_reason));
  }
  if (!fault_reason.has_value()) {
    fault_reason = RealtimeOperationalStatus::FaultReasonString(
        "Parts are faulted, but no fault reason is currently "
        "available.");
  }
  if (operational_cell_control_status->operational.state ==
          RealtimeOperationalState::kFatallyFaulted ||
      operational_cell_control_status->operational.state ==
          RealtimeOperationalState::kFaultedConnected) {
    operational_cell_control_status->operational.fault_reason = *fault_reason;
  }
  if (operational_cell_control_status->cell_control.state ==
          RealtimeOperationalState::kFatallyFaulted ||
      operational_cell_control_status->cell_control.state ==
          RealtimeOperationalState::kFaultedConnected) {
    operational_cell_control_status->cell_control.fault_reason = *fault_reason;
  }
  if (operational_cell_control_status->operational.state ==
      RealtimeOperationalState::kDisabled) {
    session_error = AbortedError("Server is disabled.");
  }

  const auto get_part_names = [this](const auto& part_indices) {
    FixedString<256> part_names("[");
    bool exceeded = false;
    for (auto part_index : part_indices) {
      if (part_names.size() > 1) {
        part_names.append(", ");
      }
      const absl::string_view part_name =
          part_manager_->GetPartName(part_index).value_or("<unknown>");
      part_names.append(part_name);
      if (part_names.size() > part_names.kMaxSize - 1 /* for closing brace*/) {
        exceeded = true;
        break;
      }
    }
    if (exceeded) {
      // Make space for "...]" so that append() works again.
      part_names.resize(part_names.kMaxSize - 4);
      part_names.append("...");
    }
    part_names.append("]");
    return part_names;
  };
  for (RtclSessionManager& session_manager : session_managers_) {
    if (!session_manager.IsActive()) {
      continue;
    }
    if (operational_cell_control_status->operational.state ==
        RealtimeOperationalState::kFatallyFaulted) {
      INTRINSIC_RT_LOG_THROTTLED(INFO)
          << "Cancelling all sessions due to fatal fault.";
      session_manager.SetErrorCleanupNotify(session_error);
      continue;
    }
    // Cancel sessions already when hardware modules fault
    // starts being propagated, not only when parts finished disabling, so the
    // safety action can run if needed.
    bool operational_available =
        operational_cell_control_status->operational.state ==
        RealtimeOperationalState::kEnabled;
    bool cell_control_available =
        operational_cell_control_status->cell_control.state ==
        RealtimeOperationalState::kEnabled;
    // Read-only sessions are not cancelled here, they have no parts.
    for (size_t part_index : session_manager.GetPartIndices()) {
      HardwareGroupSet hardware_dependencies =
          part_manager_->GetHardwareDependencies(part_index).value();
      // Cell control state is always better or equal to combined operational
      // state, so check that later.
      if (hardware_dependencies.operational_hardware &&
          !operational_available) {
        INTRINSIC_RT_LOG(INFO)
            << "Cancelling session for parts "
            << get_part_names(session_manager.GetPartIndices())
            << " due to operational hardware state '"
            << ToString(operational_cell_control_status->operational.state)
            << "'.";
        session_manager.SetErrorCleanupNotify(session_error);
        break;
      }
      if (hardware_dependencies.cell_control_hardware &&
          !cell_control_available) {
        INTRINSIC_RT_LOG(INFO)
            << "Cancelling session for parts "
            << get_part_names(session_manager.GetPartIndices())
            << " due to cell control hardware state '"
            << ToString(operational_cell_control_status->cell_control.state)
            << "'.";
        session_manager.SetErrorCleanupNotify(session_error);
        break;
      }
    }
  }
  // Our cleanup (`commit_operational_status`) automatically commits
  // ´operational_status`!
}

RealtimeStatus RealtimeControlManager::HandleOperationalStatus() {
  // Update the OperationalStatus buffer when we exit this function.
  // we don't do this at the top, because an OperationalStateCommand can
  // influence the status.
  absl::Cleanup update_operational_status = [this]() {
    UpdateOperationalStatus();
  };
  if (IsExpectedToAutomaticallyEnable()) {
    current_operational_state_command_handle_ = OperationalStateHandle{
        .handle =
            RealtimeFunctionBridge<RealtimeStatus(
                OperationalStateCommand cmd)>::RealtimeFunctionHandle{
                &auto_enable_function_data_},
        .command_id = std::nullopt,
    };
    operational_state_command_ever_set_ = true;
  }

  if (IsExpectedToAutomaticallyDisable()) {
    INTRINSIC_RT_LOG(INFO)
        << "Received new OperationalStateCommand (kDisable) from "
           "Safety.";
    current_operational_state_command_handle_ = OperationalStateHandle{
        .handle =
            RealtimeFunctionBridge<RealtimeStatus(
                OperationalStateCommand cmd)>::RealtimeFunctionHandle{
                &auto_disable_function_data_},
        .command_id = std::nullopt,
    };
    operational_state_command_ever_set_ = true;
  }

  if (!current_operational_state_command_handle_.has_value()) {
    auto handle = operational_state_command_bridge_.NextRealtimeHandle();
    if (!handle.has_value()) {
      return OkStatus();
    }
    INTRINSIC_RT_LOG(INFO) << "Received new OperationalStateCommand: "
                           << ToString(std::get<0>(*handle.value().Args()));
    current_operational_state_command_handle_ = OperationalStateHandle{
        .handle = std::move(*handle),
        .command_id = std::nullopt,
    };
    operational_state_command_ever_set_ = true;
  }
  // current_operational_state_command_handle_ definitely has a value now!
  OperationalStateCommand cmd =
      std::get<0>(*current_operational_state_command_handle_->handle.Args());
  if (!current_operational_state_command_handle_->command_id.has_value()) {
    // If there is a new handle, let the RealtimePartManager know of the
    // command.
    bool executed_command_handler =
        current_operational_state_command_handle_->handle.TryLockAndExecute(
            [this,
             &cmd](RealtimeStatus& return_value) -> RealtimeFunctionState {
              RealtimeStatusOr<std::optional<HardwareModuleManager::CommandId>>
                  command_id;
              switch (cmd) {
                case OperationalStateCommand::kClearFaults: {
                  INTRINSIC_RT_LOG(INFO) << "Requesting ClearFaults";
                  command_id = realtime_state_manager_.RequestTransition(
                      RealtimeStateManagerInterface::TransitionRequest::
                          kClearFaults);
                  break;
                }
                case OperationalStateCommand::kEnable: {
                  INTRINSIC_RT_LOG(INFO) << "Requesting EnableMotion";
                  command_id = realtime_state_manager_.RequestTransition(
                      RealtimeStateManagerInterface::TransitionRequest::
                          kEnableMotion);
                  break;
                }
                case OperationalStateCommand::kDisable: {
                  INTRINSIC_RT_LOG(INFO) << "Requesting DisableMotion";
                  command_id = realtime_state_manager_.RequestTransition(
                      RealtimeStateManagerInterface::TransitionRequest::
                          kDisableMotion);
                  break;
                }
                case OperationalStateCommand::kDisableSkipCellControlHardware: {
                  INTRINSIC_RT_LOG(INFO)
                      << "Requesting DisableMotionSkipCellControlHardware";
                  command_id = realtime_state_manager_.RequestTransition(
                      RealtimeStateManagerInterface::TransitionRequest::
                          kDisableMotionSkipCellControlHardware);
                  break;
                }
                default: {
                  return_value = InvalidArgumentError(
                      "Received invalid OperationalStateCommand, ignoring.");
                  INTRINSIC_RT_LOG(ERROR)
                      << "Returning error: " << return_value.message();
                  return RealtimeFunctionState::kDone;
                }
              }
              if (!command_id.ok()) {
                INTRINSIC_RT_LOG(ERROR)
                    << "Returning operational state command error: "
                    << command_id.status().message();
                return_value = command_id.status();

                return RealtimeFunctionState::kDone;
              }
              if (!command_id.value().has_value()) {
                return_value = OkStatus();
                INTRINSIC_RT_LOG(ERROR)
                    << "The requested command has no effect, ignoring.";
                return RealtimeFunctionState::kDone;
              }
              current_operational_state_command_handle_->command_id =
                  command_id.value();
              return RealtimeFunctionState::kRunning;
            });
    if (!executed_command_handler) {
      // Could not get the lock to execute the command, try again next cycle.
      return OkStatus();
    }
    // No command_id *in spite of having executed the command handler* means
    // that the call to TryLockAndExecute() above encountered
    // an error. In this case, we've reported that to the non-realtime thread
    // and logged it in the callback already. There's nothing more for us to
    // do here, and in particular, we don't want to tear down the
    // RtclController, so return OkStatus() instead of any error.
    if (!current_operational_state_command_handle_->command_id.has_value()) {
      INTRINSIC_RT_LOG(ERROR) << "OperationalStateCommand " << ToString(cmd)
                              << " rejected, removing handle.";
      current_operational_state_command_handle_ = std::nullopt;
      return OkStatus();
    }
  }

  // Unlike the TryLockAndExecute() above, this runs every cycle, not just on
  // the first one after the command arrives. The callback checks whether the
  // command is done. In the cycle where the command arrives, we do both
  // TryLockAndExecute()s, in case an OperationalStatus command finishes
  // immediately (for example, when trying to disable an already-disabled
  // robot).
  current_operational_state_command_handle_->handle.TryLockAndExecute(
      [this, cmd](RealtimeStatus& return_value) -> RealtimeFunctionState {
        if (!current_operational_state_command_handle_->command_id
                 .has_value()) {
          absl::string_view message = "Caller did not set a command.";
          return_value = InternalError(message);
          INTRINSIC_RT_LOG(WARNING) << message;
          return RealtimeFunctionState::kDone;
        }
        auto state = realtime_state_manager_.GetRequestState(
            *current_operational_state_command_handle_->command_id);
        if (state.ok()) {
          return CheckCommandState(*state, cmd, return_value);
        } else {
          if (state.status().code() == absl::StatusCode::kFailedPrecondition) {
            // Returns kFailedPrecondition when the command id changed in
            // the meantime, which means that another command was triggered
            // internally.
            if (realtime_state_manager_.GetRequestState() !=
                RealtimeStateManagerInterface::RequestState::kDone) {
              // Wait for the internal request to finish so that we can
              // trigger new commands.
              return RealtimeFunctionState::kRunning;
            }
            FixedString<RealtimeStatus::kMaxMessageLength> message =
                RealtimeStatus::StrCat("The command ", ToString(cmd),
                                       " got preempted internally.");
            INTRINSIC_RT_LOG(ERROR) << message;
            return_value = AbortedError(message);
          } else {
            return_value = state.status();
            INTRINSIC_RT_LOG(ERROR)
                << "RealtimeStateManager is done with non-fatal error: "
                << return_value.message();
          }
          return RealtimeFunctionState::kDone;
        }
      });
  if (current_operational_state_command_handle_->handle.Finished()) {
    INTRINSIC_RT_LOG(INFO) << "OperationalStateCommand " << ToString(cmd)
                           << " finished, removing handle.";
    current_operational_state_command_handle_ = std::nullopt;
  }

  return OkStatus();
}

RealtimeFunctionState RealtimeControlManager::CheckCommandState(
    RealtimeStateManagerInterface::RequestState request_state,
    OperationalStateCommand cmd,
    RealtimeStatus& return_value) INTRINSIC_CHECK_REALTIME_SAFE {
  switch (request_state) {
    case RealtimeStateManagerInterface::RequestState::kActive:
    case RealtimeStateManagerInterface::RequestState::kPartiallyDone:
      INTRINSIC_RT_LOG_THROTTLED(INFO) << "RealtimeStateManager is active.";
      return RealtimeFunctionState::kRunning;
      break;
    case RealtimeStateManagerInterface::RequestState::kDone: {
      // Check that we reached the correct state.
      RealtimeOperationalState op_state =
          realtime_state_manager_.GetCombinedOperationalState();
      RealtimeStatusOr<RealtimeOperationalState> expected_state =
          [cmd]() -> RealtimeStatusOr<RealtimeOperationalState> {
        switch (cmd) {
          case OperationalStateCommand::kEnable:
            return RealtimeOperationalState::kEnabled;
            break;
          case OperationalStateCommand::kDisable:
          case OperationalStateCommand::kDisableSkipCellControlHardware:
            // Operational hardware state should be disabled in both cases.
            return RealtimeOperationalState::kDisabled;
            break;
          case OperationalStateCommand::kClearFaults:
            return RealtimeOperationalState::kDisabled;
            break;
          case OperationalStateCommand::kNone:
          default:
            return InternalError(
                RealtimeStatus::StrCat("Unexpected operational state command: ",
                                       static_cast<uint32_t>(cmd)));
            break;
        }
      }();
      if (!expected_state.ok()) {
        return_value = expected_state.status();
        return RealtimeFunctionState::kDone;
      } else if (op_state == *expected_state) {
        return_value = OkStatus();
        INTRINSIC_RT_LOG(INFO)
            << "RealtimeStateManager is done without errors.";
        return RealtimeFunctionState::kDone;
      }

      if (auto fault_reason = realtime_state_manager_.GetFaultReason().value_or(
              RealtimeStateManagerInterface::FaultReasonString{});
          !fault_reason.empty()) {
        // Since the return value string size is limited, only return the
        // fault reason if available. The fault_reason should contain the most
        // useful information.
        return_value = UnknownError(fault_reason);
      } else {
        // If empty, at least return the states.
        return_value = UnknownError(RealtimeStatus::StrCat(
            "Failed to reached desired state ", ToString(*expected_state),
            ". Actual state: ", ToString(op_state), ": ", absl::string_view()));
      }
      INTRINSIC_RT_LOG(INFO)
          << "RealtimeStateManager failed with: " << return_value.message();
      return RealtimeFunctionState::kDone;
      break;
    }
    case RealtimeStateManagerInterface::RequestState::kUnknown:
    default:
      return_value = InternalError(RealtimeStatus::StrCat(
          "Unknown request state: ",
          static_cast<std::underlying_type_t<decltype(request_state)>>(
              request_state)));
      INTRINSIC_RT_LOG(ERROR)
          << "RealtimeStateManager is done with non-fatal error: "
          << return_value.message();
      return RealtimeFunctionState::kDone;  // Don't return kError since
                                            // RealtimeFunctionBridge won't
                                            // return our error message then.
      break;
  }
}

RealtimeStatus RealtimeControlManager::HandleStartSession() {
  if (!current_start_realtime_session_handle_.has_value()) {
    current_start_realtime_session_handle_ =
        start_realtime_session_bridge_.NextRealtimeHandle();
    if (!current_start_realtime_session_handle_.has_value()) {
      // Nothing to do.
      return OkStatus();
    }
  }

  const SessionId& session_id =
      std::get<0>(*current_start_realtime_session_handle_->Args());
  const absl::flat_hash_set<size_t>& part_indices =
      std::get<1>(*current_start_realtime_session_handle_->Args());
  RealtimeSessionChannels* const channels =
      std::get<2>(*current_start_realtime_session_handle_->Args());
  const RealtimeLogContext& log_context =
      std::get<3>(*current_start_realtime_session_handle_->Args());

  (void)current_start_realtime_session_handle_->TryLockAndExecute(
      [this, &part_indices, &session_id, &channels,
       &log_context](RealtimeStatus& return_value) -> RealtimeFunctionState {
        return_value = UnknownError(RealtimeStatus::StrCat(
            "RtclController failed to set a status while starting Session ",
            session_id.value()));
        std::optional<size_t> session_manager_index;
        // Sessions associated with parts have reserved slots in the beginning
        // of the `session_managers_` array. This way, there is always a slot
        // available for unused parts and they cannot be occupied by
        // monitoring sessions.
        if (!part_indices.empty()) {
          size_t min_index = *part_indices.begin();
          for (size_t index : part_indices) {
            if (RealtimeStatus safety_action_status =
                    part_manager_->CheckUnfinishedSafetyAction(index);
                !safety_action_status.ok()) {
              INTRINSIC_RT_LOG_THROTTLED(INFO)
                  << "Delaying start of session because of a safety action: "
                  << safety_action_status.ToString();
              return RealtimeFunctionState::kRunning;
            }
            if (index >= session_managers_.size()) {
              return_value = InvalidArgumentError(RealtimeStatus::StrCat(
                  "Part index ", index, " is out of range (there are only ",
                  session_managers_.size(), " Parts)."));
              return RealtimeFunctionState::kDone;
            }
            if (index < min_index) {
              min_index = index;
            }
          }
          if (session_managers_[min_index].IsActive()) {
            return_value = AlreadyExistsError(RealtimeStatus::StrCat(
                "Cannot Start Session ", session_id.value(),
                ": Channel is already in use by Session ",
                session_managers_[min_index].GetSessionId().value()));
            return RealtimeFunctionState::kDone;
          }
          session_manager_index = min_index;
        } else {
          // No parts requested -> create a monitoring session.
          // Find the first free session index in the space allocated for the
          // monitoring sessions.

          if (session_managers_.size() < kMaxNumberOfMonitoringSessions) {
            return_value = InternalError(RealtimeStatus::StrCat(
                "Session manager container is too small (size: ",
                session_managers_.size(), ", expected: ",
                kMaxNumberOfMonitoringSessions, "). This is a bug!"));
            return RealtimeFunctionState::kDone;
          }

          // Find a free session index in the section of the array reserved
          // for monitoring sessions.
          for (size_t i =
                   session_managers_.size() - kMaxNumberOfMonitoringSessions;
               i < session_managers_.size(); i++) {
            if (!session_managers_[i].IsActive()) {
              session_manager_index = i;
              break;
            }
          }

          if (!session_manager_index.has_value()) {
            return_value = ResourceExhaustedError(RealtimeStatus::StrCat(
                "No monitoring session slot available. There are only ",
                kMaxNumberOfMonitoringSessions, " slots in total."));
            return RealtimeFunctionState::kDone;
          }
        }

        if (!session_manager_index.has_value()) {
          return_value = InternalError(
              "No free session index was assigned - this is a bug! The "
              "function should return earlier in this case.");
          return RealtimeFunctionState::kDone;
        }

        // Replace the current installed Session data. Initialize it with an
        // empty SessionManager first to make sure that the cleanup in the
        // destructor is done before the new SessionManager is created.
        session_managers_.at(*session_manager_index) = RtclSessionManager();
        RealtimeStatusOr<RtclSessionManager> session_manager =
            RtclSessionManager::Create(
                session_id, *part_manager_, *channels, part_indices,
                log_context, &safety_message_handler_->SafetyStatusHandle());
        if (!session_manager.ok()) {
          return_value = session_manager.status();
          return RealtimeFunctionState::kDone;
        }
        session_managers_.at(*session_manager_index) =
            std::move(*session_manager);
        INTRINSIC_RT_LOG(INFO)
            << "Created new session at index " << *session_manager_index
            << ". Number of control sessions: "
            << CalculateUsedControlSessions()
            << ". Number of monitoring sessions: "
            << CalculateUsedMonitoringSessions()
            << ". Total max: " << session_managers_.size();
        return_value = OkStatus();
        return RealtimeFunctionState::kDone;
      });
  if (current_start_realtime_session_handle_->Finished()) {
    // We've handled the request, so delete the handle.
    current_start_realtime_session_handle_ = std::nullopt;
  }
  return OkStatus();
}

absl::Status RealtimeControlManager::Initialize(const Context& context,
                                                double control_frequency_hz) {
  // Load and apply runtime settings for this ICON resource_id.
  const std::string runtime_settings_file =
      file::JoinPath(context.GetRuntimeOptions().runtime_settings_dir,
                     absl::StrCat(context.GetRuntimeOptions().resource_id,
                                  "_runtime_settings.pb"));
  RuntimeSettings settings;
  if (auto runtime_settings = LoadRuntimeSettings(runtime_settings_file);
      runtime_settings.ok()) {
    settings = *runtime_settings;
  } else {
    LOG(INFO) << "Failed to load runtime settings from file: "
              << runtime_settings_file << ": " << runtime_settings.status()
              << ". Applying default settings.";
  }
  speed_override_buffer_ = settings.speed_override;

  // Load custom action plugins.
  if (const std::string& plugin_dir = context.GetRuntimeOptions().plugin_dir;
      !plugin_dir.empty()) {
    // Load plugins. We need to hold onto plugins_ because the dlopen'ed
    // libraries get unloaded when it goes out of scope.
    auto plugins = CustomActionPluginLoader::LoadPlugins(
        plugin_dir, RegisterActionFromPlugin);
    if (!plugins.ok()) {
      LOG(ERROR) << "PUBLIC: Plugin loading failed: " << plugins.status();
      return plugins.status();
    }
    plugins_ = std::move(plugins.value());
    if (plugins_.ListPluginBasenames().empty()) {
      LOG(INFO) << "PUBLIC: No plugins found.";
    } else {
      LOG(INFO) << "PUBLIC: Plugins loaded: "
                << absl::StrJoin(plugins_.ListPluginBasenames(), ",");
    }
  } else {
    LOG(INFO) << "PUBLIC: No plugins to be loaded.";
  }

  INTR_ASSIGN_OR_RETURN(
      safety_message_handler_,
      SafetyMessageHandler::Create(safety_message_handler_config_, context));

  server_config_.set_name(context.GetRuntimeOptions().server_name);
  server_config_.set_frequency_hz(control_frequency_hz);

  std::vector<std::string> part_names_in_index_order_;
  part_names_in_index_order_.reserve(
      parts_and_configs_.parts_with_safety_actions.size());
  std::vector<std::vector<std::string>>
      part_index_to_property_names_in_index_order;
  part_index_to_property_names_in_index_order.reserve(
      parts_and_configs_.parts_with_safety_actions.size());
  for (size_t i = 0; i < parts_and_configs_.parts_with_safety_actions.size();
       ++i) {
    const PartAndSafetyAction& part_and_safety_action =
        parts_and_configs_.parts_with_safety_actions.at(i);

    // Verify that the safety action supports all behavior overrides that are
    // configured in the SafetyMessageHandler.
    for (const auto& behavior_override :
         GetRequiredSafetyActionBehaviorOverrides()) {
      if (part_and_safety_action.safety_action
                  .supported_behavior_overrides_by_enum_value == nullptr ||
          !ActionSupportsRequestedOverrideBehavior(
              *part_and_safety_action.safety_action
                   .supported_behavior_overrides_by_enum_value,
              behavior_override)) {
        return FailedPreconditionError(RealtimeStatus::StrCat(
            "'",
            intrinsic_proto::icon::v1::BehaviorOverrideRequest_Name(
                behavior_override),
            "' not supported by safety action with ActionInstanceId ",
            part_and_safety_action.safety_action.id.value(), " of part ",
            part_and_safety_action.part.GetName()));
      }
    }

    const std::vector<PartPropertyInitialData>& part_property_data =
        parts_and_configs_.part_property_initial_data_in_index_order.at(i);
    part_names_in_index_order_.push_back(
        std::string(part_and_safety_action.part.GetName()));
    std::vector<std::string> property_names_in_index_order;
    property_names_in_index_order.reserve(part_property_data.size());
    for (const auto& property_data : part_property_data) {
      property_names_in_index_order.push_back(property_data.name);
    }
    part_index_to_property_names_in_index_order.push_back(
        std::move(property_names_in_index_order));
  }
  LOG(INFO) << "Part names in RTCL index order: ["
            << absl::StrJoin(part_names_in_index_order_, ", ") << "]";

  part_manager_ = std::make_unique<RealtimePartManager>(
      std::move(parts_and_configs_.parts_with_safety_actions),
      parts_and_configs_.part_property_initial_data_in_index_order);

  rtcl_controller_bridge_ = std::make_unique<RtclControllerBridge>(
      part_names_in_index_order_, part_index_to_property_names_in_index_order,
      part_manager_->PartPropertiesRtToNonRtBuffer(),
      part_manager_->PartPropertiesNonRtToRtBuffer(),
      start_realtime_session_bridge_, part_manager_->CurrentRobotStatusBuffer(),
      speed_override_buffer_, logging_mode_buffer_,
      part_manager_->PartStatesBuffer());

  // Now that we've set up parts and actions, initialize the RobotConnection.
  INTR_ASSIGN_OR_RETURN(
      robot_connection_,
      RobotConnectionRtclController::Create(
          server_config_, parts_and_configs_.part_configs,
          *rtcl_controller_bridge_, operational_state_,
          part_manager_->CurrentRobotStatusBuffer(), logging_mode_buffer_));

  // Finally, set up the gRPC server.
  icon_service_ = CreateApplicationLayerService(*robot_connection_);

  INTR_RETURN_IF_ERROR(StartGpioAndJoggingServers(context));

  INTR_ASSIGN_OR_RETURN(std::string topic_decimated,
                        TopicNameForRobotStatusThrottle(server_config_.name()));

  INTR_ASSIGN_OR_RETURN(std::string topic_full_rate,
                        TopicNameForRobotStatus(server_config_.name()));

  INTR_ASSIGN_OR_RETURN(
      publishing_thread_,
      CreateRealtimeCapableThread(
          ThreadOptions()
              .SetName("PublishPartStatuses")
              // The publishing thread must not run at realtime priority.
              .SetNormalPriorityAndScheduler(),
          PublishPartStatusesSaveRuntimeSettings, topic_full_rate,
          topic_decimated, control_frequency_hz, part_names_in_index_order_,
          &shutdown_publishing_thread_,
          std::ref(part_manager_->RobotStatusReader()),
          std::ref(operational_state_), std::ref(pub_sub_),
          std::ref(logging_mode_buffer_), runtime_settings_file,
          std::ref(speed_override_buffer_)),
      _ << "Failed to initialize RtclController. Could not start the part "
           "status publishing thread.");

  return absl::OkStatus();
}

void RealtimeControlManager::HandleSafetyStatus() {
  auto& safety_status = safety_message_handler_->SafetyStatusHandle();

  // Part manager exports the safety status to non realtime.
  part_manager_->SetSafetyStatus(safety_status);

  // Closes sessions on the very first cycle for systems with a safety
  // device/module since latest_mode_of_safe_operation_ is initialized to
  // UNKNOWN.
  if (const auto mode_of_safe_operation =
          safety_status.mode_of_safe_operation();
      mode_of_safe_operation != latest_mode_of_safe_operation_) {
    // ModeOfSafeOperation has changed.
    const auto error = AbortedError(RealtimeStatus::StrCat(
        "ModeOfSafeOperation changed from '",
        EnumNameModeOfSafeOperation(latest_mode_of_safe_operation_), "' to '",
        EnumNameModeOfSafeOperation(mode_of_safe_operation), "'."));
    INTRINSIC_RT_LOG(INFO) << error.message();
    latest_mode_of_safe_operation_ = mode_of_safe_operation;
    // Closes all sessions if ModeOfSafeOperation changed.
    for (RtclSessionManager& session_manager : session_managers_) {
      session_manager.SetErrorCleanupNotify(error);
    }
  }
}

bool RealtimeControlManager::IsExpectedToAutomaticallyEnable() {
  // Auto-enable when the server just started in automatic mode and no
  // operational state commands have been received yet. Also, auto-enable when
  // the physical teachpad enable button (for fully-integrated hardware) is
  // pressed in T1.
  // Never auto-enable when an OperationalState command is ongoing.
  if (current_operational_state_command_handle_.has_value()) {
    return false;
  }
  auto& safety_status = safety_message_handler_->SafetyStatusHandle();
  // Initialize to the default of UNKNOWN.
  auto mode_of_safe_operation = intrinsic_fbs::ModeOfSafeOperation::UNKNOWN;
  auto enable_button_status = intrinsic_fbs::ButtonStatus::UNKNOWN;

  mode_of_safe_operation = safety_status.mode_of_safe_operation();
  enable_button_status = safety_status.enable_button_status();

  bool should_auto_enable = false;
  switch (mode_of_safe_operation) {
    case intrinsic_fbs::ModeOfSafeOperation::CONFIGURATION:
      [[fallthrough]];
    case intrinsic_fbs::ModeOfSafeOperation::UPDATING: {
      // There is a safety system, but it's not ready.
      should_auto_enable = false;
      break;
    }
    case intrinsic_fbs::ModeOfSafeOperation::TEACH_PENDANT_1:
      [[fallthrough]];
    case intrinsic_fbs::ModeOfSafeOperation::TEACH_PENDANT_2: {
      // Enabling behavior relies on the safety system.
      bool enable_button_just_engaged =
          enable_button_status == intrinsic_fbs::ButtonStatus::ENGAGED &&
          latest_enable_button_status_ ==
              intrinsic_fbs::ButtonStatus::DISENGAGED;
      if (enable_button_just_engaged) {
        INTRINSIC_RT_LOG(INFO)
            << "Received new OperationalStateCommand (kEnable) from Safety.";
      }
      should_auto_enable = enable_button_just_engaged;
      break;
    }
    case intrinsic_fbs::ModeOfSafeOperation::UNKNOWN:
      // No safety_status.
      [[fallthrough]];
    case intrinsic_fbs::ModeOfSafeOperation::AUTOMATIC:
      [[fallthrough]];
    default: {
      // Only auto-enable when all HWMs are idle in `activated` aka `disabled`
      // and not in some transient state after a server start when no
      // operational state command has been received yet.
      should_auto_enable =
          realtime_state_manager_.GetRequestState() ==
              RealtimeStateManagerInterface::RequestState::kDone &&
          realtime_state_manager_.CheckAllInOneOfStates(
              {intrinsic_fbs::StateCode::kActivated},
              RealtimeStateManagerInterface::Group::kAllHardware) &&
          !operational_state_command_ever_set_;
    }
  }

  if (should_auto_enable) {
    INTRINSIC_RT_LOG(INFO) << "Auto-enabling ICON. State summary:\n"
                           << realtime_state_manager_.StateSummary();
  }

  return should_auto_enable;
}

bool RealtimeControlManager::IsExpectedToAutomaticallyDisable() {
  // Do not interrupt ongoing OperationalState commands (could be Enable,
  // ClearFault, or another Disable). If we did interrupt, and the ongoing
  // call was a user-generated one (i.e. ClearFaults), then that call might
  // hang indefinitely on the user side.
  if (current_operational_state_command_handle_.has_value()) {
    return false;
  }
  // Don't try to disable when there is an ongoing (internal) request.
  if (realtime_state_manager_.GetRequestState() !=
      RealtimeStateManagerInterface::RequestState::kDone) {
    return false;
  }

  const auto& safety_status = safety_message_handler_->SafetyStatusHandle();
  const auto mode_of_safe_operation = safety_status.mode_of_safe_operation();
  // Disables the robot when the `enable_button` is released in TP1 or P2 mode.
  if (!(mode_of_safe_operation ==
            intrinsic_fbs::ModeOfSafeOperation::TEACH_PENDANT_1 ||
        mode_of_safe_operation ==
            intrinsic_fbs::ModeOfSafeOperation::TEACH_PENDANT_2))
    return false;

  bool enable_button_just_disengaged =
      safety_status.enable_button_status() ==
          intrinsic_fbs::ButtonStatus::DISENGAGED &&
      latest_enable_button_status_ == intrinsic_fbs::ButtonStatus::ENGAGED;

  return enable_button_just_disengaged;
}

size_t RealtimeControlManager::CalculateUsedControlSessions() const {
  size_t control_session_count = 0;
  for (size_t i = 0;
       i < session_managers_.size() - kMaxNumberOfMonitoringSessions; i++) {
    if (session_managers_[i].IsActive()) {
      control_session_count++;
    }
  }
  return control_session_count;
}

size_t RealtimeControlManager::CalculateUsedMonitoringSessions() const {
  size_t monitoring_session_count = 0;
  for (size_t i = session_managers_.size() - kMaxNumberOfMonitoringSessions;
       i < session_managers_.size(); i++) {
    if (session_managers_[i].IsActive()) {
      monitoring_session_count++;
    }
  }
  return monitoring_session_count;
}

}  // namespace intrinsic::icon
