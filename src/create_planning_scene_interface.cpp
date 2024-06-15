#include <behaviortree_mtc/create_planning_scene_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace BT {
namespace MTC {

namespace {
constexpr auto kPortPlanningSceneInterface = "planning_scene_interface";

}  // namespace

CreatePlanningSceneInterface::CreatePlanningSceneInterface(const std::string& name,
                                                           const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreatePlanningSceneInterface::tick()
{
  auto planningSceneInterface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  setOutput(kPortPlanningSceneInterface, planningSceneInterface);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreatePlanningSceneInterface::providedPorts()
{
  return {
    BT::OutputPort<moveit::planning_interface::PlanningSceneInterfacePtr>(kPortPlanningSceneInterface, "{planning_scene_interface}"),
  };
}

}  // namespace MTC
}  // namespace BT
