#include <behaviortree_mtc/stages/create_mtc_modify_planning_scene.h>
#include <behaviortree_mtc/bt/shared_to_unique.h>

#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/robot_model/robot_model.h>

#include <behaviortree_mtc/serialization/std_containers.h>

namespace BT {
namespace MTC {

AttachDetachObjects::AttachDetachObjects(const std::string& name, const BT::NodeConfig& config, bool attach)
  : SyncActionNode(name, config), _attach(attach)
{}

BT::NodeStatus AttachDetachObjects::tick()
{
  std::string link_name;
  std::string stage_name;
  std::vector<std::string> object_names;

  if(!getInput("link_name", link_name) ||
     !getInput("stage_name", stage_name) ||
     !getInput("object_names", object_names))
    return NodeStatus::FAILURE;

  // build stage
  std::shared_ptr<moveit::task_constructor::stages::ModifyPlanningScene> stage{ nullptr };
  stage = {
    new moveit::task_constructor::stages::ModifyPlanningScene(stage_name),
    dirty::fake_deleter{}
  };

  stage->attachObjects(object_names, link_name, _attach);

  // upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList AttachDetachObjects::providedPorts()
{
  return {
    BT::InputPort<std::string>("stage_name"),
    BT::InputPort<std::vector<std::string>>("object_names", "list of object names/id's."),
    BT::InputPort<std::vector<std::string>>("link_name", "attach or detach a list of objects to the given link"),
    BT::OutputPort<moveit::task_constructor::StagePtr>("stage", "ModifyPlanningScene stage"),
  };
}

AllowForbidCollisions::AllowForbidCollisions(const std::string& name, const BT::NodeConfig& config, bool allow)
  : SyncActionNode(name, config), _allow(allow)
{}

BT::NodeStatus AllowForbidCollisions::tick(const std::vector<std::string>& first, const std::vector<std::string>& second)
{
  std::string stage_name;

  if(!getInput("stage_name", stage_name))
    return NodeStatus::FAILURE;

  // build stage
  std::shared_ptr<moveit::task_constructor::stages::ModifyPlanningScene> stage{ nullptr };
  stage = {
    new moveit::task_constructor::stages::ModifyPlanningScene(stage_name),
    dirty::fake_deleter{}
  };

  stage->allowCollisions(first, second, _allow);

  // upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList AllowForbidCollisions::providedPorts()
{
  return {
    BT::InputPort<std::string>("stage_name"),
    BT::OutputPort<moveit::task_constructor::StagePtr>("stage", "ModifyPlanningScene stage"),
  };
}

AllowForbidCollisionPairs::AllowForbidCollisionPairs(const std::string& name, const BT::NodeConfig& config, bool allow)
  : AllowForbidCollisions(name, config, allow)
{}

BT::NodeStatus AllowForbidCollisionPairs::tick()
{
  std::shared_ptr<std::vector<std::string>> first_names;
  std::shared_ptr<std::vector<std::string>> second_names;

  if(!getInput("first", first_names) ||
     !getInput("second", second_names))
    return NodeStatus::FAILURE;

  return AllowForbidCollisions::tick(*first_names, *second_names);
}

BT::PortsList AllowForbidCollisionPairs::providedPorts()
{
  auto port_lists = AllowForbidCollisions::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<std::vector<std::string>>>("first", "first object of collision pair"));
  port_lists.emplace(BT::InputPort<std::shared_ptr<std::vector<std::string>>>("second", "second object of collision pair"));

  return port_lists;
}

AllowForbidAllCollisions::AllowForbidAllCollisions(const std::string& name, const BT::NodeConfig& config, bool allow)
  : AllowForbidCollisions(name, config, allow)
{}

BT::NodeStatus AllowForbidAllCollisions::tick()
{
  std::shared_ptr<std::vector<std::string>> objects;

  if(!getInput("objects", objects))
    return NodeStatus::FAILURE;

  return AllowForbidCollisions::tick(*objects, std::vector<std::string>());
}

BT::PortsList AllowForbidAllCollisions::providedPorts()
{
  auto port_lists = AllowForbidCollisions::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<std::vector<std::string>>>("objects", "allow/forbid all collisions for given object/s"));

  return port_lists;
}

AllowForbidCollisionsJointModelGroup::AllowForbidCollisionsJointModelGroup(const std::string& name, const BT::NodeConfig& config, bool allow)
  : AllowForbidCollisions(name, config, allow)
{}

BT::NodeStatus AllowForbidCollisionsJointModelGroup::tick()
{
  std::shared_ptr<std::vector<std::string>> objects;
  moveit::task_constructor::TaskPtr task;
  std::string jmg_name;

  if(!getInput("objects", objects) ||
     !getInput("jmg_name", jmg_name) ||
     !getInput("task", task))
    return NodeStatus::FAILURE;

  if(auto jmg = task->getRobotModel()->getJointModelGroup(jmg_name))
  {
    const auto& links = jmg->getLinkModelNamesWithCollisionGeometry();

    return AllowForbidCollisions::tick(*objects, links);
  }
  return NodeStatus::FAILURE;
}

BT::PortsList AllowForbidCollisionsJointModelGroup::providedPorts()
{
  auto port_lists = AllowForbidCollisions::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<std::vector<std::string>>>("objects", "allow/forbid collisions with joint model group links"));
  port_lists.emplace(BT::InputPort<std::string>("jmg_name", "joint model group name"));
  port_lists.emplace(BT::InputPort<moveit::task_constructor::TaskPtr>("task", "MTC task"));

  return port_lists;
}

}  // namespace MTC
}  // namespace BT
