#include <behaviortree_mtc/mtc_allow_collisions.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/stages/modify_planning_scene.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

namespace
{
constexpr auto kPortMainObject = "main_object";
constexpr auto kPortOtherObjects = "other_objects";
constexpr auto kPortObject = "object";
constexpr auto kPortStage = "stage";
constexpr auto kPortAllowCollisions = "allow_collisions";
constexpr auto kPortStageName = "stage_name";
}  // namespace

MTCAllowCollisionsOneToMany::MTCAllowCollisionsOneToMany(const std::string& name,
                                                         const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus MTCAllowCollisionsOneToMany::tick()
{
  // Retrieve inputs
  bool allow_collisions;
  std::string main_object, name;
  std::vector<std::string> other_objects;

  if(!getInput(kPortStageName, name) ||
     !getInput(kPortMainObject, main_object) ||
     !getInput(kPortAllowCollisions, allow_collisions) ||
     !getInput(kPortOtherObjects, other_objects))
    return NodeStatus::FAILURE;

  // Build stage
  std::shared_ptr<MTC::stages::ModifyPlanningScene> stage{ nullptr };
  stage = {
    new MTC::stages::ModifyPlanningScene(name),
    dirty::fake_deleter{}
  };
  stage->allowCollisions(
      main_object, other_objects,
      allow_collisions);

  // Upcast to base class
  MTC::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList MTCAllowCollisionsOneToMany::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortMainObject, "object allowed/forbided to collide with other objects"),
    BT::InputPort<std::vector<std::string>>(kPortOtherObjects, "objects allowed/forbided to collide with the main_object"),
    BT::InputPort<std::string>(kPortStageName),
    BT::InputPort<bool>(kPortAllowCollisions, "True/False"),
    BT::OutputPort<MTC::StagePtr>(kPortStage),
  };
}

MTCAllowCollisionsOnOneObject::MTCAllowCollisionsOnOneObject(const std::string& name,
                                                             const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus MTCAllowCollisionsOnOneObject::tick()
{
  // Retrieve inputs
  bool allow_collisions;
  std::string object, name;

  if(!getInput(kPortStageName, name) ||
     !getInput(kPortObject, object) ||
     !getInput(kPortAllowCollisions, allow_collisions))
    return NodeStatus::FAILURE;
    
  // Build stage
  std::shared_ptr<MTC::stages::ModifyPlanningScene> stage{ nullptr };
  stage = {
    new MTC::stages::ModifyPlanningScene(name),
    dirty::fake_deleter{}
  };
  stage->allowCollisions(
      object,
      allow_collisions);

  // Upcast to base class
  MTC::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList MTCAllowCollisionsOnOneObject::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortObject, "object allowed/forbided to collide with other objects"),
    BT::InputPort<std::string>(kPortStageName),
    BT::InputPort<bool>(kPortAllowCollisions, "True/False"),
    BT::OutputPort<MTC::StagePtr>(kPortStage),
  };
}