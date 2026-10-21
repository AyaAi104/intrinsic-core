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

#include "intrinsic/world/objects/test/object_world_test_utils.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/api/io.h"
#include "intrinsic/geometry/api/shape_factory.h"
#include "intrinsic/geometry/storage/dummy_storage.h"
#include "intrinsic/geometry/storage/geometry_serializer.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/resources/proto/geometric_resource_data.pb.h"
#include "intrinsic/scene/conversion/object_properties_conversion.h"
#include "intrinsic/util/proto/parse_text_proto.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/component/attachment_component.h"
#include "intrinsic/world/component/collections_component.h"
#include "intrinsic/world/component/collections_member_component.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/component/kinematics_component.h"
#include "intrinsic/world/component/robot_component.h"
#include "intrinsic/world/dof_kinematic_view.h"
#include "intrinsic/world/entity.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/geometry_types.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/labels.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_creation_utils.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/world_object_internal.h"
#include "intrinsic/world/proto/collections_component.pb.h"
#include "intrinsic/world/proto/kinematics_component.pb.h"
#include "intrinsic/world/test/world_test_utils.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace object_world {
namespace {

constexpr absl::string_view k3DofRobotTextProto = R"pb(
  name: "3dof_robot"
  entities {
    name: "base_link"
    link {}
  }
  entities {
    name: "A1"
    parent_name: "base_link"
    parent_t_this {
      position { z: 0.4 }
      orientation { w: 1 }
    }
    joint {
      kinematics_component {
        motion_type: MOTION_TYPE_REVOLUTE
        axis { z: -1 }
        parent_t_inboard {
          position { z: 0.4 }
          orientation { w: 1 }
        }
        system_limits {
          fixed_limits { lower: -10 upper: 12 }
          velocity: 1.1
          acceleration: 2.2
          jerk: 20
        }
      }
    }
  }
  entities {
    name: "A1_link"
    parent_name: "A1"
    link {}
  }
  entities {
    name: "A2"
    parent_name: "A1_link"
    parent_t_this {
      position { x: 0.025 }
      orientation { w: 0.70710678118654757 y: 0.70710678118654757 }
    }
    joint {
      kinematics_component {
        motion_type: MOTION_TYPE_REVOLUTE
        axis { y: 1 }
        parent_t_inboard {
          position { x: 0.025 }
          orientation { w: 0.70710678118654757 y: 0.70710678118654757 }
        }
        system_limits {
          fixed_limits { lower: -4 upper: 5.2 }
          velocity: 2.1
          acceleration: 7.2
          jerk: 15
        }
      }
    }
  }
  entities {
    name: "A2_link"
    parent_name: "A2"
    link {}
  }
  entities {
    name: "A3"
    parent_name: "A2_link"
    parent_t_this {
      position { z: 0.455 }
      orientation { w: 0.70710678118654757 y: -0.70710678118654757 }
    }
    joint {
      kinematics_component {
        motion_type: MOTION_TYPE_REVOLUTE
        axis { z: 1 }
        parent_t_inboard {
          position { z: 0.455 }
          orientation { w: 0.70710678118654757 y: -0.70710678118654757 }
        }
        system_limits {
          fixed_limits { lower: -2 upper: 2.2 }
          velocity: 6.1
          acceleration: 3.2
          jerk: 10
        }
      }
    }
  }
  entities {
    name: "A3_link"
    parent_name: "A3"
    link {}
  }
)pb";

constexpr absl::string_view kPinchGripperTextProto = R"pb(
  name: "pinch_gripper"
  entities {
    name: "base_link"
    link {}
  }
  entities {
    name: "joint_0"
    parent_name: "base_link"
    joint {
      kinematics_component {
        motion_type: MOTION_TYPE_PRISMATIC
        axis { z: 1 }
      }
    }
  }
  entities {
    name: "finger_0"
    parent_name: "joint_0"
    link {
      geometry_component {
        named_geometries {
          key: "Intrinsic_Collision"
          value {
            named_geometries {
              key: "0"
              value {
                geometry {
                  inline_geometry_data {
                    exact_geometry {
                      primitive_set {
                        primitives {
                          shape { box { size { x: 0.1 y: 0.1 z: 0.1 } } }
                        }
                      }
                    }
                  }
                }
                ref_t_shape {
                  matrix4d {
                    rows: 4
                    cols: 4
                    values: [ 1, 0, 0, 0 ]
                    values: [ 0, 1, 0, 0 ]
                    values: [ 0, 0, 1, 0 ]
                    values: [ 0, 0, 0, 1 ]
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  entities {
    name: "joint_1"
    parent_name: "base_link"
    joint {
      kinematics_component {
        motion_type: MOTION_TYPE_PRISMATIC
        axis { z: 1 }
      }
    }
  }
  entities {
    name: "finger_1"
    parent_name: "joint_1"
    link {
      geometry_component {
        named_geometries {
          key: "Intrinsic_Collision"
          value {
            named_geometries {
              key: "0"
              value {
                geometry {
                  inline_geometry_data {
                    exact_geometry {
                      primitive_set {
                        primitives {
                          shape { box { size { x: 0.1 y: 0.1 z: 0.1 } } }
                        }
                      }
                    }
                  }
                }
                ref_t_shape {
                  matrix4d {
                    rows: 4
                    cols: 4
                    values: [ 1, 0, 0, 0 ]
                    values: [ 0, 1, 0, 0 ]
                    values: [ 0, 0, 1, 0 ]
                    values: [ 0, 0, 0, 1 ]
                  }
                }
              }
            }
          }
        }
      }
    }
  }
)pb";

void UpdateIfFinite(double in, double& out) {
  if (std::isfinite(in)) {
    out = in;
  }
}

absl::StatusOr<AttachmentEntityId> GetSingleLeafEntityOfCollection(
    const World& world,
    const std::set<CollectionsMemberEntityId>& collection_member_ids,
    WorldObjectName parent_name) {
  WorldHashSet<AttachmentEntityId> attachment_ids;
  for (auto collection_member_id : collection_member_ids) {
    INTR_ASSIGN_OR_RETURN(
        AttachmentEntityId attachment_id,
        world.ValidateEntity<AttachmentEntityId>(collection_member_id));
    attachment_ids.insert(attachment_id);
  }

  WorldHashSet<AttachmentEntityId> leaf_candidate_ids = attachment_ids;
  for (AttachmentEntityId member_id : attachment_ids) {
    INTR_ASSIGN_OR_RETURN(
        const AttachmentComponent* attachment,
        world.GetComponentByEntityId<AttachmentComponent>(member_id));
    if (attachment_ids.contains(attachment->GetParentId())) {
      leaf_candidate_ids.erase(attachment->GetParentId());
    }
  }

  if (leaf_candidate_ids.size() != 1) {
    return absl::InternalError(absl::StrCat(
        "Cannot attach to object with name \"", parent_name.value(),
        "\" since the object neither consists of exactly one attachment entity "
        "nor is it a linear chain of attachment entities with a single leaf "
        "entity."));
  }
  return *leaf_candidate_ids.begin();
}

absl::StatusOr<CollectionsEntityId> GetCollectionsEntityIdForObjectName(
    const WorldObjectName& object_name, const World& world) {
  for (CollectionsEntityId collection_id :
       world.GetTypedEntityIds<CollectionsEntityId>()) {
    INTR_ASSIGN_OR_RETURN(const WorldEntity* collection_entity,
                          world.GetEntityById(collection_id));
    if (collection_entity->GetAlias() == object_name.value()) {
      return collection_id;
    }
  }
  return absl::InvalidArgumentError(absl::Substitute(
      "Cannot find object with name \"$0\"", object_name.value()));
}

absl::StatusOr<std::vector<AttachmentEntityId>>
GetCollectionMembersForObjectName(const WorldObjectName& object_name,
                                  const World& world) {
  if (object_name == RootObjectName()) {
    return std::vector<AttachmentEntityId>{kRootEntityId};
  }

  for (CollectionsEntityId collection_id :
       world.GetTypedEntityIds<CollectionsEntityId>()) {
    INTR_ASSIGN_OR_RETURN(const WorldEntity* collection_entity,
                          world.GetEntityById(collection_id));
    if (collection_entity->GetAlias() == object_name.value()) {
      INTR_ASSIGN_OR_RETURN(
          const CollectionsComponent* collections,
          collection_entity->GetComponent<CollectionsComponent>());
      std::vector<AttachmentEntityId> result;
      for (CollectionsMemberEntityId member_id :
           collections->GetAllCollectionMembers()) {
        INTR_ASSIGN_OR_RETURN(
            AttachmentEntityId member_attachment_id,
            world.ValidateEntity<AttachmentEntityId>(member_id));
        result.push_back(member_attachment_id);
      }
      return result;
    }
  }
  return absl::InvalidArgumentError(absl::Substitute(
      "Cannot find object with name \"$0\"", object_name.value()));
}

struct AttachmentInfo {
  AttachmentEntityId parent_id;
  Pose3d parent_t_new_entity;
};

absl::StatusOr<AttachmentInfo> ComputeAttachmentToParentObject(
    WorldObjectName parent_name, Pose3d parent_root_t_new_entity,
    const World& world) {
  if (parent_name == RootObjectName()) {
    return AttachmentInfo{kRootEntityId, parent_root_t_new_entity};
  }
  // Search for parent object with the given parent name, i.e., search for a
  // collection which has that name as an alias.
  for (CollectionsEntityId collection_id :
       world.GetTypedEntityIds<CollectionsEntityId>()) {
    INTR_ASSIGN_OR_RETURN(const WorldEntity* collection_entity,
                          world.GetEntityById(collection_id));
    if (collection_entity->GetAlias() == parent_name.value()) {
      AttachmentInfo result;
      INTR_ASSIGN_OR_RETURN(
          const CollectionsComponent* collection,
          collection_entity->GetComponent<CollectionsComponent>());

      // Ignore frames -- these can be explicitly set by using the
      // `parent_frame_name` parameter... except for attachment frames, which
      // are used to explicitly extend the kinematic chain.
      const std::vector<CollectionsMemberEntityId>& coordinate_frame_members =
          collection->GetCollectionMembers(
              CollectionsComponent::kCoordinateFrames);
      const std::vector<CollectionsMemberEntityId>& attachment_frame_members =
          collection->GetCollectionMembers(
              CollectionsComponent::kAttachmentFrames);
      std::set<CollectionsMemberEntityId> members;
      for (auto member : collection->GetAllCollectionMembers()) {
        const bool is_coordinate_frame =
            absl::c_find(coordinate_frame_members, member) !=
            coordinate_frame_members.end();
        const bool is_attachment_frame =
            absl::c_find(attachment_frame_members, member) !=
            attachment_frame_members.end();
        if (!is_coordinate_frame || is_attachment_frame) {
          members.insert(member);
        }
      }
      INTR_ASSIGN_OR_RETURN(
          AttachmentEntityId parent_root_id,
          world.GetRootEntity({members.begin(), members.end()}));
      INTR_ASSIGN_OR_RETURN(result.parent_id, GetSingleLeafEntityOfCollection(
                                                  world, members, parent_name));

      // Parent object can consist of multiple entities and we are attaching
      // to the leaf entity. Correct for the fact that the returned attachment
      // pose has to be relative to the parent's leaf (the entity we are
      // attaching to) and that the given 'params.parent_t_this' is relative
      // to the parent's root (the objects origin).
      Pose3d parent_leaf_t_parent_root =
          world.GetTransform(result.parent_id, parent_root_id);
      result.parent_t_new_entity =
          parent_leaf_t_parent_root * parent_root_t_new_entity;
      return result;
    }
  }
  return absl::InternalError(
      absl::StrCat("Could not find parent entity for object with name \n",
                   parent_name.value(), "\n"));
}

absl::StatusOr<AttachmentInfo> ComputeAttachmentToParentFrame(
    WorldObjectName parent_object_name, FrameName parent_frame_name,
    Pose3d parent_frame_t_new_entity, const World& world) {
  INTR_ASSIGN_OR_RETURN(
      std::vector<AttachmentEntityId> member_ids,
      GetCollectionMembersForObjectName(parent_object_name, world));
  INTR_ASSIGN_OR_RETURN(std::vector<AttachmentEntityId> frame_ids,
                        GetChildFrameEntitiesRecursively(
                            world, {member_ids.begin(), member_ids.end()},
                            /*attachment_graph=*/nullptr));
  for (AttachmentEntityId frame_id : frame_ids) {
    INTR_ASSIGN_OR_RETURN(const WorldEntity* frame_entity,
                          world.GetEntityById(frame_id));
    if (frame_entity->GetLocalName() == parent_frame_name.value()) {
      AttachmentInfo result;
      result.parent_id = frame_id;
      result.parent_t_new_entity = parent_frame_t_new_entity;
      return result;
    }
  }

  return absl::InvalidArgumentError(absl::Substitute(
      "Cannot find frame with name \"$0\" under object with name \"$1\"",
      parent_frame_name.value(), parent_object_name.value()));
}

absl::Status SetParentAndParentTThis(
    const WorldObjectName& parent_name,
    std::optional<AttachmentEntityId> attach_to_object_entity,
    const std::optional<Pose3d>& parent_t_this,
    const std::optional<Pose3d>& object_entity_t_this, const World& world,
    CreateEntityParams& create_entity_params) {
  if (parent_t_this && object_entity_t_this) {
    return absl::InvalidArgumentError(
        "'parent_t_this' and 'object_entity_t_this' cannot be used at the same "
        "time.");
  }

  if (attach_to_object_entity) {
    INTR_ASSIGN_OR_RETURN(
        std::vector<AttachmentEntityId> parent_entities,
        GetCollectionMembersForObjectName(parent_name, world));
    if (absl::c_find(parent_entities, *attach_to_object_entity) ==
        parent_entities.end()) {
      return absl::InvalidArgumentError(absl::Substitute(
          "Could not attach new object to entity $0. The entity is not part of "
          "the given parent object \"$1\".",
          attach_to_object_entity->value(), parent_name.value()));
    }

    create_entity_params.parent = *attach_to_object_entity;

    if (object_entity_t_this) {
      create_entity_params.parent_t_this =
          object_entity_t_this.value_or(Pose3d());
    } else if (parent_t_this) {
      INTR_ASSIGN_OR_RETURN(AttachmentEntityId parent_root_id,
                            world.GetRootEntity({parent_entities.begin(),
                                                 parent_entities.end()}));
      Pose3d attach_t_parent_root =
          world.GetTransform(*attach_to_object_entity, parent_root_id);
      create_entity_params.parent_t_this =
          attach_t_parent_root * *parent_t_this;
    } else {
      create_entity_params.parent_t_this = Pose3d();
    }
  } else {
    // 'attach_to_object_entity' not set, attach to final entity (default).
    if (object_entity_t_this) {
      return absl::InvalidArgumentError(
          "'object_entity_t_this' can only be used in conjunction with "
          "'attach_to_object_entity' which has not been set.");
    }
    INTR_ASSIGN_OR_RETURN(
        AttachmentInfo attachment,
        ComputeAttachmentToParentObject(
            parent_name,
            /*parent_root_t_new_entity=*/parent_t_this.value_or(Pose3d()),
            world));
    create_entity_params.parent = attachment.parent_id;
    create_entity_params.parent_t_this = attachment.parent_t_new_entity;
  }
  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<ObjectEntitiesInfo> CreateEntitiesForObject(
    World* world, const CreateEntitiesForObjectParameters& params) {
  const bool single_entity_mode = !params.collision_geo.empty() ||
                                  !params.visual_geo.empty() ||
                                  !params.geometry.empty();
  const bool multi_entity_mode = !params.linear_chain_entities.empty() ||
                                 params.num_linear_chain_entities > 0;

  if (single_entity_mode && multi_entity_mode) {
    return absl::InvalidArgumentError(
        "CreateEntitiesForObject: Cannot use 'collision_geo', 'visual_geo' or "
        "'geometry' in combination with 'linear_chain_entities' or "
        "'num_linear_chain_entities'.");
  }
  if (!params.linear_chain_entities.empty() &&
      params.num_linear_chain_entities > 0) {
    return absl::InvalidArgumentError(
        "CreateEntitiesForObject: You can only use one of "
        "'linear_chain_entities' or 'num_linear_chain_entities'.");
  }

  std::vector<EntityForObjectParameters> linear_chain_entities;
  if (multi_entity_mode) {
    if (!params.linear_chain_entities.empty()) {
      linear_chain_entities = params.linear_chain_entities;
    } else if (params.num_linear_chain_entities > 0) {
      linear_chain_entities.resize(params.num_linear_chain_entities);
    } else {
      return absl::InternalError(
          "Invalid multi_entity_mode in CreateEntitiesForObject().");
    }
  } else {
    // single_entity_mode
    linear_chain_entities.push_back(EntityForObjectParameters{
        .visual_geo = std::move(params.visual_geo),
        .collision_geo = std::move(params.collision_geo),
        .geometry = std::move(params.geometry),
    });
  }

  INTR_ASSIGN_OR_RETURN(
      EntityId collection_id,
      CreateEntity(
          world,
          {
              .local_name = params.name.value(),
              .alias = params.name_is_global_alias ? params.name.value() : "",
              .create_empty_collection = true,
              .resource_name = params.resource_name,
              .make_static = params.make_static,
              .user_data_protos = std::move(params.user_data),
          }));
  CollectionsEntityId typed_collection_id(collection_id.value());

  std::vector<EntityId> link_ids;
  std::vector<TypedEntityId<AttachmentComponentType, GeometryComponentType,
                            CollectionsMemberComponentType>>
      link_ids_typed;
  link_ids.reserve(linear_chain_entities.size());
  link_ids_typed.reserve(linear_chain_entities.size());

  for (const EntityForObjectParameters& entity_params : linear_chain_entities) {
    if (link_ids.empty() && entity_params.parent_entity_t_entity.has_value()) {
      return absl::InvalidArgumentError(
          "CreateEntitiesForObject: You cannot specify "
          "'parent_entity_t_entity' for the first entity.");
    }

    std::string local_name = absl::StrCat("link_", link_ids.size());

    CreateEntityParams link_params;
    link_params.local_name = local_name;
    link_params.add_as_link_to_collections.push_back(typed_collection_id);
    if (!entity_params.geometry.empty()) {
      link_params.geometry = entity_params.geometry;
    } else if (!entity_params.collision_geo.empty() ||
               !entity_params.visual_geo.empty()) {
      link_params.collision_geo = entity_params.collision_geo;
      link_params.visual_geo = entity_params.visual_geo;
    } else {
      link_params.geometry = {{"0", MakeTransformedCenteredBox(0.1, 0.1, 0.1)}};
    }
    link_params.collision_exclusions = std::vector<EntityId>();
    link_params.make_default_physics = true;

    if (link_ids.empty()) {
      INTR_RETURN_IF_ERROR(SetParentAndParentTThis(
          params.parent_name, params.attach_to_object_entity,
          params.parent_t_this, params.object_entity_t_this, *world,
          link_params));
    } else {
      link_params.parent = AttachmentEntityId(link_ids.back());
      link_params.parent_t_this =
          entity_params.parent_entity_t_entity.value_or(Pose3d());
    }

    INTR_ASSIGN_OR_RETURN(EntityId link_id,
                          CreateEntity(world, std::move(link_params)));
    link_ids.push_back(link_id);
    link_ids_typed.emplace_back(link_id);
  }

  std::vector<EntityId> sensor_ids;
  std::vector<TypedEntityId<AttachmentComponentType, SensorComponentType>>
      sensor_ids_typed;
  TypedEntityId<AttachmentComponentType, SensorComponentType> sensor_id_typed(
      kInvalidEntityId);

  if (params.add_camera_sensor_with_object_t_sensor.has_value() +
          params.add_depth_camera_sensor_with_object_t_sensor.has_value() +
          !params.add_camera_multi_sensor_with_object_t_sensors.empty() >
      1) {
    return absl::InvalidArgumentError(
        "Do not use more than one of "
        "{'add_camera_sensor_with_object_t_sensor', "
        "'add_depth_camera_sensor_with_object_t_sensor', "
        "'add_camera_multi_sensor_with_object_t_sensors'} simultaneously.");
  }

  if (params.add_camera_sensor_with_object_t_sensor.has_value() ||
      !params.add_camera_multi_sensor_with_object_t_sensors.empty()) {
    const std::vector<Pose3d> sensor_poses =
        params.add_camera_sensor_with_object_t_sensor.has_value()
            ? std::vector<Pose3d>{params.add_camera_sensor_with_object_t_sensor
                                      .value()}
            : params.add_camera_multi_sensor_with_object_t_sensors;

    for (int i = 0; i < sensor_poses.size(); ++i) {
      INTR_ASSIGN_OR_RETURN(
          EntityId sensor_id,
          CreateEntity(
              world, {
                         .local_name = absl::StrCat("camera_sensor_", i),
                         // attach to root link entity
                         .parent = link_ids_typed.front(),
                         .parent_t_this = sensor_poses[i],
                         .make_camera_sensor = true,
                         .add_as_sensor_to_collections = {typed_collection_id},
                         .resource_name = params.resource_name,
                     }));
      sensor_ids.push_back(sensor_id);
      sensor_id_typed =
          TypedEntityId<AttachmentComponentType, SensorComponentType>(
              sensor_id.value());
      sensor_ids_typed.push_back(sensor_id_typed);
    }
  }

  if (params.add_depth_camera_sensor_with_object_t_sensor.has_value()) {
    INTR_ASSIGN_OR_RETURN(
        EntityId sensor_id,
        CreateEntity(
            world, {
                       .local_name = "camera_sensor",
                       // attach to root link entity
                       .parent = link_ids_typed.front(),
                       .parent_t_this =
                           *params.add_depth_camera_sensor_with_object_t_sensor,
                       .make_depth_camera_sensor = true,
                       .add_as_sensor_to_collections = {typed_collection_id},
                       .resource_name = params.resource_name,
                   }));
    sensor_ids.push_back(sensor_id);
    sensor_id_typed =
        TypedEntityId<AttachmentComponentType, SensorComponentType>(
            sensor_id.value());
    sensor_ids_typed.push_back(sensor_id_typed);
  }

  return ObjectEntitiesInfo{.collection_id = typed_collection_id,
                            .link_id = link_ids_typed.front(),
                            .link_ids = link_ids_typed,
                            .sensor_id = sensor_id_typed,
                            .sensor_ids = sensor_ids_typed};
}

absl::StatusOr<LinearChainRobotData> Create3DofRobotObject(
    World* world, const Create3DofRobotObjectParameters& params) {
  static int id = 0;
  LinearChainRobotParams entity_robot_params =
      CreateSimpleAssemblyLinearChainRobotParams(
          absl::StrCat("irrelevant_robot_group_id_", id++));
  entity_robot_params.robot_alias = params.name.value();
  entity_robot_params.resource_name = params.resource_name;

  if (params.joint_limits) {
    for (int i = 0; i < entity_robot_params.joint_params.size(); ++i) {
      UpdateIfFinite(params.joint_limits->min_position[i],
                     entity_robot_params.joint_params[i].lower_value_limit);
      UpdateIfFinite(params.joint_limits->max_position[i],
                     entity_robot_params.joint_params[i].upper_value_limit);
      UpdateIfFinite(params.joint_limits->max_velocity[i],
                     entity_robot_params.joint_params[i].velocity_limit);
      UpdateIfFinite(params.joint_limits->max_acceleration[i],
                     entity_robot_params.joint_params[i].acceleration_limit);
      UpdateIfFinite(params.joint_limits->max_jerk[i],
                     entity_robot_params.joint_params[i].jerk_limit);
      UpdateIfFinite(params.joint_limits->max_torque[i],
                     entity_robot_params.joint_params[i].effort_limit);
    }
  }

  INTR_ASSIGN_OR_RETURN(
      AttachmentInfo attachment,
      ComputeAttachmentToParentObject(
          params.parent_name,
          /*parent_root_t_new_entity=*/params.parent_object_t_object, *world));
  entity_robot_params.parent_id = attachment.parent_id;
  entity_robot_params.parent_t_base_link = attachment.parent_t_new_entity;
  entity_robot_params.add_coordinate_frame = params.create_flange_frame;
  entity_robot_params.add_solver_key = params.add_solver_key;

  INTR_ASSIGN_OR_RETURN(LinearChainRobotData result,
                        CreateLinearChainRobot(world, entity_robot_params));
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<DofKinematicView> dof_view,
                        world->GetDofKinematicView(result.robot_id));
  INTR_RETURN_IF_ERROR(
      dof_view->SetDofValues(params.joint_positions, /*enforce_limits=*/true));

  if (params.create_flange_frame) {
    if (result.frame_ids.size() != 1) {
      return absl::InternalError(
          absl::Substitute("Set `create_flange_frame` but created $0 frames!",
                           result.frame_ids.size()));
    }
    INTR_ASSIGN_OR_RETURN(WorldEntity * flange_entity,
                          world->GetEntityById(*result.frame_ids.begin()));
    INTR_RETURN_IF_ERROR(
        flange_entity->SetLocalName(FlangeFrameName().value()));
  }

  if (!params.named_joint_configurations.empty()) {
    for (const auto& [_, joint_position] : params.named_joint_configurations) {
      if (dof_view->GetDofCount() != joint_position.size()) {
        return absl::InvalidArgumentError(absl::Substitute(
            "'named_joint_configurations' contained a joint position with an "
            "invalid DoF count. Received $0 DoFs, expected $1 DoFs.",
            joint_position.size(), dof_view->GetDofCount()));
      }
    }

    INTR_ASSIGN_OR_RETURN(
        RobotComponent * component,
        world->GetComponentByEntityId<RobotComponent>(result.robot_id));
    component->SetNamedConfigurations(
        WorldHashMap<std::string, eigenmath::VectorXd>(
            params.named_joint_configurations));
  }

  if (params.cartesian_limits.has_value()) {
    INTR_ASSIGN_OR_RETURN(
        RobotComponent * component,
        world->GetComponentByEntityId<RobotComponent>(result.robot_id));

    INTR_RETURN_IF_ERROR(
        component->SetCartesianLimits(*params.cartesian_limits));
  }

  if (params.mounted_payload.has_value()) {
    INTR_ASSIGN_OR_RETURN(
        RobotComponent * component,
        world->GetComponentByEntityId<RobotComponent>(result.robot_id));

    INTR_RETURN_IF_ERROR(component->SetMountedPayload(*params.mounted_payload));
  }

  return result;
}

absl::StatusOr<intrinsic_proto::scene_object::v1::SceneObject>
Create3DofRobotSceneObject(const Create3DofRobotObjectParameters& params) {
  auto scene_object =
      intrinsic::ParseTextOrDie<intrinsic_proto::scene_object::v1::SceneObject>(
          k3DofRobotTextProto);

  scene_object.set_name(params.name.value());

  // Find A1, A2, A3 joints in the parsed proto to apply joint_positions and
  // joint_limits
  intrinsic_proto::world::KinematicsComponent* A1_kin = nullptr;
  intrinsic_proto::world::KinematicsComponent* A2_kin = nullptr;
  intrinsic_proto::world::KinematicsComponent* A3_kin = nullptr;

  for (auto& entity : *scene_object.mutable_entities()) {
    if (entity.name() == "A1") {
      A1_kin = entity.mutable_joint()->mutable_kinematics_component();
    } else if (entity.name() == "A2") {
      A2_kin = entity.mutable_joint()->mutable_kinematics_component();
    } else if (entity.name() == "A3") {
      A3_kin = entity.mutable_joint()->mutable_kinematics_component();
    }
  }

  CHECK(A1_kin != nullptr);
  CHECK(A2_kin != nullptr);
  CHECK(A3_kin != nullptr);

  A1_kin->set_raw_value(params.joint_positions[0]);
  A2_kin->set_raw_value(params.joint_positions[1]);
  A3_kin->set_raw_value(params.joint_positions[2]);

  if (params.joint_limits) {
    std::vector<intrinsic_proto::world::KinematicsComponent*> joints = {
        A1_kin, A2_kin, A3_kin};
    for (int i = 0; i < 3; ++i) {
      if (std::isfinite(params.joint_limits->min_position[i])) {
        joints[i]->mutable_system_limits()->mutable_fixed_limits()->set_lower(
            params.joint_limits->min_position[i]);
      }
      if (std::isfinite(params.joint_limits->max_position[i])) {
        joints[i]->mutable_system_limits()->mutable_fixed_limits()->set_upper(
            params.joint_limits->max_position[i]);
      }
      if (std::isfinite(params.joint_limits->max_velocity[i])) {
        joints[i]->mutable_system_limits()->set_velocity(
            params.joint_limits->max_velocity[i]);
      }
      if (std::isfinite(params.joint_limits->max_acceleration[i])) {
        joints[i]->mutable_system_limits()->set_acceleration(
            params.joint_limits->max_acceleration[i]);
      }
      if (std::isfinite(params.joint_limits->max_jerk[i])) {
        joints[i]->mutable_system_limits()->set_jerk(
            params.joint_limits->max_jerk[i]);
      }
      if (std::isfinite(params.joint_limits->max_torque[i])) {
        joints[i]->mutable_system_limits()->set_effort(
            params.joint_limits->max_torque[i]);
      }
    }
  }

  if (params.create_flange_frame) {
    auto* flange = scene_object.add_entities();
    flange->set_name("flange");
    flange->set_parent_name("A3_link");
    flange->mutable_frame();
  }

  if (params.add_solver_key) {
    auto* solver = scene_object.mutable_properties()
                       ->mutable_kinematics()
                       ->add_ik_solvers();
    solver->set_ik_solver("kinematic_chain");
  }

  if (params.cartesian_limits) {
    *scene_object.mutable_properties()->mutable_kinematics()->mutable_limits() =
        intrinsic::scene_object::ToProto(*params.cartesian_limits);
  }

  for (const auto& [config_name, joint_positions] :
       params.named_joint_configurations) {
    auto* named_config = scene_object.mutable_properties()
                             ->mutable_kinematics()
                             ->add_named_configurations();
    named_config->set_name(config_name);
    (*named_config->mutable_joint_positions())["A1"] = joint_positions[0];
    (*named_config->mutable_joint_positions())["A2"] = joint_positions[1];
    (*named_config->mutable_joint_positions())["A3"] = joint_positions[2];
  }

  return scene_object;
}

absl::StatusOr<intrinsic_proto::resources::GeometricResourceInstanceData>
CreateResourceInstanceDataFor3DofRobotObject(
    const Create3DofRobotObjectParameters& params) {
  if (params.resource_name.empty()) {
    return absl::InvalidArgumentError(
        "Cannot create ResourceInstanceData without a resource name.");
  }
  INTR_ASSIGN_OR_RETURN(auto scene_object, Create3DofRobotSceneObject(params));
  intrinsic_proto::resources::GeometricResourceInstanceData
      resource_instance_data;
  resource_instance_data.set_name(params.resource_name);
  *resource_instance_data.mutable_scene_object() = scene_object;
  return resource_instance_data;
}

absl::StatusOr<PinchGripperData> CreatePinchGripperObject(
    World* world, const CreatePinchGripperObjectParameters& params) {
  // Define a kinematic object similar to a two-finger gripper which doesn't
  // have a flange frame attached to it.
  //
  // Entity visualization:
  //   base (link) --> joint_left  (prismatic joint) --> finger_left  (link)
  //               \-> joint_right (prismatic joint) --> finger_right (link)
  NamedGeometrySet some_geometry = {
      {"0", MakeTransformedCenteredBox(0.1, 0.1, 0.1)}};

  CreateEntityParams root_link_params{
      .local_name = "base",
      .geometry = some_geometry,
      .collision_exclusions = std::vector<EntityId>{},
      .make_default_physics = true,
  };
  INTR_RETURN_IF_ERROR(SetParentAndParentTThis(
      params.parent_name, params.attach_to_object_entity, params.parent_t_this,
      params.object_entity_t_this, *world, root_link_params));

  INTR_ASSIGN_OR_RETURN(EntityId base_id,
                        CreateEntity(world, root_link_params));
  INTR_ASSIGN_OR_RETURN(
      EntityId joint_left_id,
      CreateEntity(world, {.local_name = "joint_left",
                           .parent = AttachmentEntityId{base_id},
                           .make_prismatic_joint = true}));
  INTR_ASSIGN_OR_RETURN(
      EntityId joint_right_id,
      CreateEntity(world, {.local_name = "joint_right",
                           .parent = AttachmentEntityId{base_id},
                           .make_prismatic_joint = true}));
  INTR_ASSIGN_OR_RETURN(
      EntityId finger_left_id,
      CreateEntity(world, {.local_name = "finger_left",
                           .parent = AttachmentEntityId{joint_left_id},
                           .geometry = some_geometry}));
  INTR_ASSIGN_OR_RETURN(
      EntityId finger_right_id,
      CreateEntity(world, {.local_name = "finger_right",
                           .parent = AttachmentEntityId{joint_right_id},
                           .geometry = some_geometry}));
  INTR_ASSIGN_OR_RETURN(
      CollectionsEntityId collection_id,
      CreateEntity<CollectionsEntityId>(
          world, {.alias = params.name.value(),
                  .create_collection_with_members =
                      {.links = {base_id, finger_left_id, finger_right_id},
                       .joints = {joint_left_id, joint_right_id}},
                  .make_default_robot = true,
                  .resource_name = params.resource_name}));
  return PinchGripperData{.collection_id = collection_id,
                          .base_id = base_id,
                          .finger_left_id = finger_left_id,
                          .finger_right_id = finger_right_id};
}

absl::StatusOr<AttachmentEntityId> CreateEntityForFrame(
    World* world, const CreateEntityForFrameParameters& params) {
  CreateEntityParams entity_params;
  entity_params.local_name = params.name.value();

  const bool attach_to_object_entity =
      params.attach_to_object_entity.has_value();
  const bool attach_to_frame = !params.parent_frame_name.value().empty();

  if (!attach_to_object_entity && !attach_to_frame) {
    INTR_ASSIGN_OR_RETURN(
        AttachmentInfo attachment,
        ComputeAttachmentToParentObject(
            params.parent_name,
            /*parent_root_t_new_entity=*/params.parent_t_this, *world));
    entity_params.parent = attachment.parent_id;
    entity_params.parent_t_this = attachment.parent_t_new_entity;
  } else if (!attach_to_object_entity && attach_to_frame) {
    INTR_ASSIGN_OR_RETURN(
        AttachmentInfo attachment,
        ComputeAttachmentToParentFrame(
            params.parent_name, params.parent_frame_name,
            /*parent_frame_t_new_entity=*/params.parent_t_this, *world));
    entity_params.parent = attachment.parent_id;
    entity_params.parent_t_this = attachment.parent_t_new_entity;
  } else if (attach_to_object_entity && !attach_to_frame) {
    INTR_ASSIGN_OR_RETURN(
        std::vector<AttachmentEntityId> entities,
        GetCollectionMembersForObjectName(params.parent_name, *world));
    if (absl::c_find(entities, *params.attach_to_object_entity) ==
        entities.end()) {
      return absl::InvalidArgumentError(absl::Substitute(
          "CreateEntityForFrame: Could not attach new frame to entity $0. The "
          "entity is not part of the given parent object \"$1\".",
          params.attach_to_object_entity->value(), params.parent_name.value()));
    }
    entity_params.parent = *params.attach_to_object_entity;
    entity_params.parent_t_this = params.parent_t_this;
  } else {
    return absl::InvalidArgumentError(
        "CreateEntityForFrame: You can only use one of "
        "'attach_to_object_entity' or 'parent_frame_name'.");
  }

  if (params.parent_name == RootObjectName()) {
    if (params.mark_as_attachment_frame) {
      return absl::InvalidArgumentError(
          "The root object does not have a collections entity, so cannot have "
          "atachment frames.");
    }
  } else {
    INTR_ASSIGN_OR_RETURN(
        auto collection_id,
        GetCollectionsEntityIdForObjectName(params.parent_name, *world));
    entity_params.add_as_coordinate_frame_to_collections.push_back(
        collection_id);
    if (params.mark_as_attachment_frame) {
      entity_params.add_as_attachment_frame_to_collections.push_back(
          collection_id);
    }
  }

  INTR_ASSIGN_OR_RETURN(EntityId entity_id,
                        CreateEntity(world, std::move(entity_params)));
  return AttachmentEntityId(entity_id.value());
}

absl::StatusOr<intrinsic_proto::scene_object::v1::SceneObject>
CreateSceneObjectForPinchGripper(double upper_joint_limit,
                                 double lower_joint_limit) {
  auto scene_object =
      intrinsic::ParseTextOrDie<intrinsic_proto::scene_object::v1::SceneObject>(
          kPinchGripperTextProto);

  for (auto& entity : *scene_object.mutable_entities()) {
    if (entity.name() == "joint_0" || entity.name() == "joint_1") {
      auto* kin = entity.mutable_joint()->mutable_kinematics_component();
      kin->mutable_system_limits()->mutable_fixed_limits()->set_lower(
          lower_joint_limit);
      kin->mutable_system_limits()->mutable_fixed_limits()->set_upper(
          upper_joint_limit);
    }
  }

  return scene_object;
}

absl::StatusOr<intrinsic_proto::scene_object::v1::SceneObject>
CreateSceneObjectForSuctionGripper() {
  intrinsic_proto::scene_object::v1::SceneObject scene_object;
  scene_object.set_name(std::string(kGripperLinkElementName));
  auto* entity = scene_object.add_entities();
  entity->set_name(std::string(kGripperLinkElementName));
  auto* link = entity->mutable_link();

  INTR_ASSIGN_OR_RETURN(
      auto geo_proto,
      intrinsic::ToProto(MakeTransformedCenteredBox(0.1, 0.1, 0.1), nullptr));
  auto* named_geometries =
      link->mutable_geometry_component()->mutable_named_geometries();
  auto& geometry_set = (*named_geometries)[intrinsic::kKindCollisionGeometry];
  (*geometry_set.mutable_named_geometries())["0"] = std::move(geo_proto);

  return scene_object;
}

JointLimitsXd GetDefault7DofLimits() {
  JointLimitsXd limits;
  limits.min_position = eigenmath::VectorXd::Constant(7, -3.0);
  limits.max_position = eigenmath::VectorXd::Constant(7, 3.0);
  limits.max_velocity = eigenmath::VectorXd::Constant(7, 2.0);
  limits.max_acceleration = eigenmath::VectorXd::Constant(7, 100.0);
  limits.max_jerk =
      eigenmath::VectorXd::Constant(7, std::numeric_limits<double>::max());
  limits.max_torque = eigenmath::VectorXd::Constant(7, 100.0);
  return limits;
}

absl::StatusOr<World> CreateTestWorldWith7DofRobot(
    absl::string_view robot_name, const JointLimitsXd& application_limits) {
  if (!application_limits.IsSizeConsistent() ||
      application_limits.min_position.size() != 7) {
    return absl::InvalidArgumentError(
        "application_limits must be size-consistent and have 7 DoFs.");
  }

  intrinsic_proto::scene_object::v1::SceneObject scene_object;
  scene_object.set_name(std::string(robot_name));

  auto* base_link = scene_object.add_entities();
  base_link->set_name("base_link");
  base_link->mutable_link();

  struct JointSpec {
    absl::string_view joint_name;
    absl::string_view parent_link_name;
    absl::string_view child_link_name;
    Pose3d parent_t_inboard;
  };
  // Synthetic 7-DoF spherical-shoulder-elbow-wrist serial manipulator with
  // alternating Z/Y revolute joint axes and distinct link lengths (0.21 m,
  // 0.18 m, 0.23 m, 0.17 m, 0.26 m, 0.14 m, and 0.09 m).
  const std::vector<JointSpec> joint_specs = {
      {"joint_1", "base_link", "link_1",
       Pose3d(eigenmath::Vector3d(0, 0, 0.21))},
      {"joint_2", "link_1", "link_2",
       Pose3d(eigenmath::Quaterniond(0, 0, M_SQRT1_2, M_SQRT1_2),
              eigenmath::Vector3d(0, 0, 0.18))},
      {"joint_3", "link_2", "link_3",
       Pose3d(eigenmath::Quaterniond(0, 0, M_SQRT1_2, M_SQRT1_2),
              eigenmath::Vector3d(0, 0.23, 0))},
      {"joint_4", "link_3", "link_4",
       Pose3d(eigenmath::Quaterniond(M_SQRT1_2, M_SQRT1_2, 0, 0),
              eigenmath::Vector3d(0, 0, 0.17))},
      {"joint_5", "link_4", "link_5",
       Pose3d(eigenmath::Quaterniond(0, 0, M_SQRT1_2, M_SQRT1_2),
              eigenmath::Vector3d(0, 0.26, 0))},
      {"joint_6", "link_5", "link_6",
       Pose3d(eigenmath::Quaterniond(M_SQRT1_2, M_SQRT1_2, 0, 0),
              eigenmath::Vector3d(0, 0, 0.14))},
      {"joint_7", "link_6", "link_7",
       Pose3d(eigenmath::Quaterniond(0, 0, M_SQRT1_2, M_SQRT1_2),
              eigenmath::Vector3d(0, 0.09, 0))},
  };

  const JointLimitsXd default_system_limits = GetDefault7DofLimits();
  for (size_t i = 0; i < joint_specs.size(); ++i) {
    const JointSpec& spec = joint_specs[i];
    auto* joint = scene_object.add_entities();
    joint->set_name(std::string(spec.joint_name));
    joint->set_parent_name(std::string(spec.parent_link_name));
    *joint->mutable_parent_t_this() = intrinsic::ToProto(spec.parent_t_inboard);
    auto* kinematics = joint->mutable_joint()->mutable_kinematics_component();
    kinematics->set_motion_type(
        intrinsic_proto::world::KinematicsComponent::MOTION_TYPE_REVOLUTE);
    *kinematics->mutable_axis() =
        intrinsic::ToVectorProto(eigenmath::Vector3d::UnitZ());
    *kinematics->mutable_parent_t_inboard() =
        intrinsic::ToProto(spec.parent_t_inboard);
    auto* system_limits = kinematics->mutable_system_limits();
    system_limits->mutable_fixed_limits()->set_lower(
        std::min(default_system_limits.min_position[i],
                 application_limits.min_position[i]));
    system_limits->mutable_fixed_limits()->set_upper(
        std::max(default_system_limits.max_position[i],
                 application_limits.max_position[i]));
    system_limits->set_velocity(std::max(default_system_limits.max_velocity[i],
                                         application_limits.max_velocity[i]));
    system_limits->set_acceleration(
        std::max(default_system_limits.max_acceleration[i],
                 application_limits.max_acceleration[i]));
    system_limits->set_jerk(std::max(default_system_limits.max_jerk[i],
                                     application_limits.max_jerk[i]));
    system_limits->set_effort(std::max(default_system_limits.max_torque[i],
                                       application_limits.max_torque[i]));

    auto* app_limits = kinematics->mutable_application_limits();
    app_limits->mutable_fixed_limits()->set_lower(
        application_limits.min_position[i]);
    app_limits->mutable_fixed_limits()->set_upper(
        application_limits.max_position[i]);
    app_limits->set_velocity(application_limits.max_velocity[i]);
    app_limits->set_acceleration(application_limits.max_acceleration[i]);
    app_limits->set_jerk(application_limits.max_jerk[i]);
    app_limits->set_effort(application_limits.max_torque[i]);

    auto* link = scene_object.add_entities();
    link->set_name(std::string(spec.child_link_name));
    link->set_parent_name(std::string(spec.joint_name));
    link->mutable_link();
  }

  auto* flange = scene_object.add_entities();
  flange->set_name(FlangeFrameName().value());
  flange->set_parent_name("link_7");
  flange->mutable_frame();

  auto* ik_solver =
      scene_object.mutable_properties()->mutable_kinematics()->add_ik_solvers();
  ik_solver->set_ik_solver("kinematic_chain");
  ik_solver->set_tip_link_name("link_7");

  CartesianLimits cart_limits;
  cart_limits.max_rotational_jerk = 10.0;
  cart_limits.max_rotational_acceleration = 10.0;
  cart_limits.max_rotational_velocity = 1.0;
  cart_limits.max_translational_position.setConstant(10.0);
  cart_limits.min_translational_position.setConstant(-10.0);
  cart_limits.max_translational_velocity.setConstant(1.0);
  cart_limits.min_translational_velocity.setConstant(-1.0);
  cart_limits.max_translational_acceleration.setConstant(10.0);
  cart_limits.min_translational_acceleration.setConstant(-10.0);
  cart_limits.max_translational_jerk.setConstant(10.0);
  cart_limits.min_translational_jerk.setConstant(-10.0);
  *scene_object.mutable_properties()->mutable_kinematics()->mutable_limits() =
      intrinsic::scene_object::ToProto(cart_limits);

  World world = World::CreateEmptyWorld();
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<ObjectWorld> object_world,
                        ObjectWorld::CreateView(world));
  INTR_ASSIGN_OR_RETURN(WorldObject * root_object,
                        object_world->GetObject(RootObjectName()));
  INTR_ASSIGN_OR_RETURN(
      WorldObject * robot_object,
      root_object->CreateChildObject(
          WorldObjectName(robot_name), WorldObjectNameType::kNameIsGlobalAlias,
          Pose3d::Identity(), /*geolib=*/nullptr, scene_object));
  INTR_RETURN_IF_ERROR(robot_object->SetResourceName(robot_name));
  INTR_ASSIGN_OR_RETURN(
      const KinematicObject* kinematic_robot,
      object_world->GetKinematicObject(WorldObjectName(robot_name)));
  INTR_ASSIGN_OR_RETURN(AttachmentEntityId tip_id,
                        world.GetFinalEntityOfRobotKinematicChain(
                            kinematic_robot->GetRobotEntityId()));
  INTR_ASSIGN_OR_RETURN(WorldEntity * tip_entity, world.GetEntityById(tip_id));
  INTR_RETURN_IF_ERROR(tip_entity->AddLabels({labels::Tip()}));
  return world;
}

}  // namespace object_world
}  // namespace intrinsic
