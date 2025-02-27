#include <behaviortree_mtc/moveit/add_object_to_planning_scene.h>
#include <behaviortree_mtc/bt/shared_to_unique.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

namespace BT {
namespace MTC {

AddObjectToPlanningScene::AddObjectToPlanningScene(const std::string& name,
                                                   const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus AddObjectToPlanningScene::tick()
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

BT::PortsList AddObjectToPlanningScene::providedPorts()
{
  return {
    BT::InputPort<moveit::planning_interface::PlanningSceneInterfacePtr>("planning_scene_interface"),
    BT::InputPort<std::shared_ptr<moveit_msgs::CollisionObject>>("collision_object"),
  };
}

}  // namespace MTC
}  // namespace BT
