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

#ifndef INTRINSIC_WORLD_OBJECTS_TEST_OBJECT_WORLD_TEST_UTILS_H_
#define INTRINSIC_WORLD_OBJECTS_TEST_OBJECT_WORLD_TEST_UTILS_H_

#include <gmock/gmock.h>

#include <optional>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/resources/proto/geometric_resource_data.pb.h"
#include "intrinsic/scene/proto/v1/scene_object.pb.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/hashing/hashing.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/robot_payload/robot_payload.h"
#include "intrinsic/world/test/world_test_utils.h"
#include "intrinsic/world/world.h"

namespace intrinsic {
namespace object_world {

// Defines what link elements hold children (collisions) that are contact
// points for a grip.
inline constexpr absl::string_view kGripperLinkElementName = "gripper_link";

// Parameters for a single link entity in CreateEntitiesForObjectParameters
// below.
struct EntityForObjectParameters {
  // Required geometry of the created link entity. There are three options:
  // - Set at least one of {'visual_geo', 'collision_geo'}.
  // - Set 'geometry' to set both visual and collision geometry at the same
  //   time.
  // - Default: Set nothing and the entities visual and collision geometry will
  //   be a (0.1m)³ box centered at the entities origin.
  NamedGeometrySet visual_geo;
  NamedGeometrySet collision_geo;
  NamedGeometrySet geometry;

  // Pose of the entity in the space of the parent entity. DO NOT SPECIFY this
  // for the first/root entity of an object as the pose of the first/root entity
  // of an object is determined through
  // 'CreateEntitiesForObjectParameters.parent_object_t_object'.
  std::optional<Pose3d> parent_entity_t_entity;
};

// Parameter struct for CreateEntitiesForObject below.
struct CreateEntitiesForObjectParameters {
  WorldObjectName name;

  bool name_is_global_alias = true;

  // Name of the parent object to which the new object shall be attached. By
  // default (if 'attach_to_object_entity' is not given), the newly created
  // object will be attached to the final link of this parent object .
  WorldObjectName parent_name = RootObjectName();

  // If present, the newly created object will be attached to the given entity
  // of the parent object (see 'parent_name') instead of being attached to its
  // final link.
  std::optional<AttachmentEntityId> attach_to_object_entity;

  // Pose of the new object's base/origin (= the created "link" entity) in the
  // space of the parent object's base/origin (= the root entity of the parent
  // object). Do not combine with 'object_entity_t_this'.
  std::optional<Pose3d> parent_t_this;

  // Pose of the new object's base/origin (= the created "link" entity) in the
  // space of the parent entity as specified by 'attach_to_object_entity' (which
  // needs to be set). Do not combine with 'parent_t_this'.
  std::optional<Pose3d> object_entity_t_this;

  // Required geometry of the created object (if creating an object with a
  // single link entity). There are three options:
  // - Set at least one of {'visual_geo', 'collision_geo'}.
  // - Set 'geometry' to set both visual and collision geometry at the same
  //   time.
  // - Default: Set nothing and the objects visual and collision geometry will
  //   be a (0.1m)³ box centered at the objects origin.
  //
  // Do not combine with 'linear_chain_entities' or 'num_linear_chain_entities'.
  NamedGeometrySet visual_geo;
  NamedGeometrySet collision_geo;
  NamedGeometrySet geometry;

  // If not empty, an object with multiple entities in a "linear chain" will be
  // created. The first element specifies the root link entity, the second
  // element specifies the child of the root link entity and so on.
  //
  // Do not combine with 'visual_geo', 'collision_geo', 'geometry', or
  // 'num_linear_chain_entities'.
  std::vector<EntityForObjectParameters> linear_chain_entities;

  // If set to a value >0, an object with 'n' entities in a "linear chain" will
  // be created. This is equivalent to:
  //   .linear_chain_entities = {{}, ..., {}} // with n elements
  //
  // Do not combine with 'visual_geo', 'collision_geo', 'geometry', or
  // 'linear_chain_entities'.
  int num_linear_chain_entities = 0;

  // If set, an entity with a default Camera-type sensor will be added to the
  // created object having the given pose in the space of the object. The
  // created sensor entity is in addition to the link entities created by
  // default or through 'linear_chain_entities' or 'num_linear_chain_entities'.
  std::optional<Pose3d> add_camera_sensor_with_object_t_sensor;

  // If set, an entity with a default DepthCamera-type sensor will be added to
  // the created object having the given pose in the space of the object. The
  // created sensor entity is in addition to the link entities created by
  // default or through 'linear_chain_entities' or 'num_linear_chain_entities'.
  std::optional<Pose3d> add_depth_camera_sensor_with_object_t_sensor;

  // If set, multiple entities with a default Camera-type sensor will be added
  // to the created object having the given poses in the space of the object.
  // The created sensor entities are in addition to the link entities created by
  // default or through 'linear_chain_entities' or 'num_linear_chain_entities'.
  std::vector<Pose3d> add_camera_multi_sensor_with_object_t_sensors;

  // If not empty, the new object will be associated with the given resource
  // name.
  absl::string_view resource_name;

  // If true, a SimulationComponent will be created on the collection entity of
  // the object which has the "IsStatic" property set (all other properties have
  // the default value).
  bool make_static = false;

  // If not empty, the new object will be associated with the given user data.
  WorldHashMap<std::string, google::protobuf::Any> user_data;
};

struct ObjectEntitiesInfo {
  CollectionsEntityId collection_id;

  // The id of the object's single link entity (if creating one link) or the
  // root link entity (if creating multiple links). Equal to the first element
  // of 'link_ids' below.
  //
  // The entity id-type encodes the minimal set of components. Depending on the
  // values passed with CreateEntitiesForObjectParameters, the link entity might
  // have additional components.
  TypedEntityId<AttachmentComponentType, GeometryComponentType,
                CollectionsMemberComponentType>
      link_id;

  // The ids of the object's link entities sorted from root to leaves.
  //
  // The entity id-type encodes the minimal set of components. Depending on the
  // values passed with CreateEntitiesForObjectParameters, the link entity might
  // have additional components.
  std::vector<TypedEntityId<AttachmentComponentType, GeometryComponentType,
                            CollectionsMemberComponentType>>
      link_ids;

  // The id of the object's sensor entity. Equal to the first element
  // of 'sensor_ids' below. Equal to kInvalidEntityId if there is
  // no sensor entity.
  TypedEntityId<AttachmentComponentType, SensorComponentType> sensor_id;

  // The ids of the object's sensor entities.
  std::vector<TypedEntityId<AttachmentComponentType, SensorComponentType>>
      sensor_ids;
};

// Creates a configurable physical object consisting of one or more "link"
// entities which contain the object's geometry etc. and one collection entity
// representing the object as a whole.
//
// Assumes that the given world is already compatible with the object-based view
// (see ObjectWorld) and guarantees that this compatibility is maintained.
absl::StatusOr<ObjectEntitiesInfo> CreateEntitiesForObject(
    World* world, const CreateEntitiesForObjectParameters& params);

// Parameter struct for Create3DofRobotObject below.
struct Create3DofRobotObjectParameters {
  WorldObjectName name;

  // Name of the parent object to which the new object shall be attached. If the
  // referenced object consists of more than one attachment entity the newly
  // created object will be attached to the leaf of those attachment entities.
  // If there is more than one leaf an error will be returned. E.g., if the
  // parent object is a linear chain robot, the newly created robot will be
  // attached to the parent robot's final link.
  WorldObjectName parent_name = RootObjectName();

  // Pose of the robot's base/origin (=the robot's base link) in the space of
  // the parent object's base/origin.
  Pose3d parent_object_t_object;

  // Initial position of the robot.
  eigenmath::Vector3d joint_positions = eigenmath::Vector3d(0, 0, 0);

  // If present, initial joint limits of the robots that should replace the
  // defaults.
  std::optional<JointLimitsXd> joint_limits;

  // The named joint configurations stored with the created robot.
  WorldHashMap<std::string, eigenmath::VectorXd> named_joint_configurations;

  // If not empty, the robot will be associated with the given resource name.
  absl::string_view resource_name;

  // If true, a flange frame will be added as a child of the robot as required
  // by the object-world view. If set to false, such a frame has to be manually
  // added later.
  bool create_flange_frame = true;

  // Optional cartesian limits to store into the robot component.
  std::optional<CartesianLimits> cartesian_limits = std::nullopt;

  // Optional mounted payload to store into the robot component.
  std::optional<RobotPayload> mounted_payload = std::nullopt;

  // If true an IK solver key will be added to the robot component.
  bool add_solver_key = false;
};

// Creates the entities for a robot object with a linear chain of 4-links and
// 3-joints (cf. CreateLinearChainRobot() and
// CreateSimpleAssemblyLinearChainRobotParams()).
// Assumes that the given world is already compatible with the object-based view
// (see ObjectWorld) and guarantees that this compatibility is maintained.
absl::StatusOr<LinearChainRobotData> Create3DofRobotObject(
    World* world, const Create3DofRobotObjectParameters& params);

absl::StatusOr<intrinsic_proto::scene_object::v1::SceneObject>
Create3DofRobotSceneObject(const Create3DofRobotObjectParameters& params);

// Creates the GeometricResourceInstanceData for a robot object similar to
// Create3DofRobotObject for resource instance composition tests.
absl::StatusOr<intrinsic_proto::resources::GeometricResourceInstanceData>
CreateResourceInstanceDataFor3DofRobotObject(
    const Create3DofRobotObjectParameters& params);

// Parameter struct for CreatePinchGrippers below.
struct CreatePinchGripperObjectParameters {
  WorldObjectName name;

  // Name of the parent object to which the new gripper shall be attached. By
  // default (if 'attach_to_object_entity' is not given), the newly created
  // object will be attached to the final link of this parent object .
  WorldObjectName parent_name = RootObjectName();

  // If present, the newly created object will be attached to the given entity
  // of the parent object (see 'parent_name') instead of being attached to its
  // final link.
  std::optional<AttachmentEntityId> attach_to_object_entity;

  // Pose of the new object's base/origin (= the created "link" entity) in the
  // space of the parent object's base/origin (= the root entity of the parent
  // object). Do not combine with 'object_entity_t_this'.
  std::optional<Pose3d> parent_t_this;

  // Pose of the new object's base/origin (= the created "link" entity) in the
  // space of the parent entity (see 'attach_to_object_entity'). Can only be
  // used in conjunction with  'attach_to_object_entity'. Do not combine with
  // 'parent_t_this'.
  std::optional<Pose3d> object_entity_t_this;

  // If not empty, the new object will be associated with the given resource
  // handle name;
  absl::string_view resource_name;
};

struct PinchGripperData {
  // The entity id of the gripper collection.
  CollectionsEntityId collection_id;

  // The entity id of the gripper's base link.
  EntityId base_id;
  // The entity id of the gripper's left finger link.
  EntityId finger_left_id;
  // The entity id of the gripper's right finger link.
  EntityId finger_right_id;
};

// Creates the entities for a pinch gripper with two prismatic joints.
absl::StatusOr<PinchGripperData> CreatePinchGripperObject(
    World* world, const CreatePinchGripperObjectParameters& params);

// Parameter struct for CreateEntityForFrame below.
struct CreateEntityForFrameParameters {
  FrameName name;

  // Name of the parent object under which the new frame shall be grouped. If
  // 'attach_to_object_entity' and 'parent_frame_name' below are not specified,
  // the newly created frame will by default be attached to the leaf attachment
  // entity of the parent object. If there is more than one leaf an error will
  // be returned. E.g., if the parent object is a linear chain robot, the newly
  // created frame will by default be attached to the robots final link.
  WorldObjectName parent_name = RootObjectName();

  // If present, the newly created frame will be attached to the given entity of
  // the parent object (see 'parent_name'). Do not combine with
  // 'parent_frame_name'.
  std::optional<AttachmentEntityId> attach_to_object_entity;

  // If non-empty, the newly created frame will be attached to the frame with
  // the given name under the parent object (see 'parent_name'). Do not combine
  // with 'attach_to_object_entity'.
  FrameName parent_frame_name = FrameName("");

  // Pose of this frame in the space of
  // - the parent object, by default,
  // - the parent entity, if 'attach_to_object_entity' is specified,
  // - the parent frame, if 'parent_frame_name' is specified.
  Pose parent_t_this;

  // When attaching to the object, mark this frame as part of the attachment
  // frame collection in the parent object.
  bool mark_as_attachment_frame = false;
};

// Creates a configurable frame consisting of one attachment entity. This
// entity is itself not part of a collection, but needs to be a child of an
// "object" (i.e. a collection created e.g. with CreateEntitiesForObject()
// above).
// Assumes that the given world is already compatible with the object-based view
// (see ObjectWorld) and guarantees that this compatibility is maintained.
absl::StatusOr<AttachmentEntityId> CreateEntityForFrame(
    World* world, const CreateEntityForFrameParameters& params);

absl::StatusOr<intrinsic_proto::scene_object::v1::SceneObject>
CreateSceneObjectForPinchGripper(double upper_joint_limit = 1.0,
                                 double lower_joint_limit = 0.0);

absl::StatusOr<intrinsic_proto::scene_object::v1::SceneObject>
CreateSceneObjectForSuctionGripper();

// Returns the default joint limits for the 7-DoF test robot in
// `CreateTestWorldWith7DofRobot()`.
JointLimitsXd GetDefault7DofLimits();

// Creates a test world containing a synthetic 7-DoF serial manipulator robot
// named `robot_name` with alternating Z/Y revolute joints (`joint_1` through
// `joint_7`), links (`base_link` and `link_1` through `link_7` with distinct
// link lengths), a `flange` frame attached to `link_7`, default Cartesian
// limits, and the specified joint `application_limits`.
absl::StatusOr<World> CreateTestWorldWith7DofRobot(
    absl::string_view robot_name = "my_robot",
    const JointLimitsXd& application_limits = GetDefault7DofLimits());

// Matches a container which contains the expected_elements in the given order
// but migh also contain any additional elements. 'expected_elements' has to be
// an iterable container of matchers for the elements in 'arg'.
MATCHER_P(ContainsInOrder, expected_elements,
          std::string(negation ? "doesn't" : "does") + " contain in order " +
              PrintToString(expected_elements)) {
  auto expected_elements_iter = expected_elements.begin();
  auto arg_iter = arg.begin();
  while (expected_elements_iter != expected_elements.end() &&
         arg_iter != arg.end()) {
    if (expected_elements_iter->Matches(*arg_iter)) {
      ++expected_elements_iter;
    }
    ++arg_iter;
  }
  return expected_elements_iter == expected_elements.cend();
}

}  // namespace object_world
}  // namespace intrinsic

#endif  // INTRINSIC_WORLD_OBJECTS_TEST_OBJECT_WORLD_TEST_UTILS_H_
