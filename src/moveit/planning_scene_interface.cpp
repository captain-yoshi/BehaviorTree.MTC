#include <behaviortree_mtc/moveit/planning_scene_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace BT {
namespace MTC {

PlanningSceneInterface::PlanningSceneInterface(const std::string& name,
                                               const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus PlanningSceneInterface::tick()
{
  auto planningSceneInterface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  setOutput("planning_scene_interface", planningSceneInterface);

  return NodeStatus::SUCCESS;
}

BT::PortsList PlanningSceneInterface::providedPorts()
{
  return {
    BT::OutputPort<moveit::planning_interface::PlanningSceneInterfacePtr>("planning_scene_interface", "{planning_scene_interface}"),
  };
}

PlanningSceneInterfaceAddCollisionObject::PlanningSceneInterfaceAddCollisionObject(const std::string& name,
                                                                                   const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus PlanningSceneInterfaceAddCollisionObject::tick()
{
  moveit::planning_interface::PlanningSceneInterfacePtr psi;
  std::shared_ptr<moveit_msgs::CollisionObject> object;

  //INPUTS
  if(!getInput("planning_scene_interface", psi) ||
     !getInput("collision_object", object))
    return NodeStatus::FAILURE;
  //APPLY OBJECT
  if(!psi->applyCollisionObject(*object))
    return NodeStatus::FAILURE;
  return NodeStatus::SUCCESS;
}

BT::PortsList PlanningSceneInterfaceAddCollisionObject::providedPorts()
{
  return {
    BT::InputPort<moveit::planning_interface::PlanningSceneInterfacePtr>("planning_scene_interface"),
    BT::InputPort<std::shared_ptr<moveit_msgs::CollisionObject>>("collision_object"),
  };
}

}  // namespace MTC
}  // namespace BT
