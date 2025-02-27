#include <behaviortree_mtc/moveit/create_planning_scene_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace BT {
namespace MTC {

CreatePlanningSceneInterface::CreatePlanningSceneInterface(const std::string& name,
                                                           const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreatePlanningSceneInterface::tick()
{
  auto planningSceneInterface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  setOutput("planning_scene_interface", planningSceneInterface);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreatePlanningSceneInterface::providedPorts()
{
  return {
    BT::OutputPort<moveit::planning_interface::PlanningSceneInterfacePtr>("planning_scene_interface", "{planning_scene_interface}"),
  };
}

}  // namespace MTC
}  // namespace BT
