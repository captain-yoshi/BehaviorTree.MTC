#include <behaviortree_mtc/stages/connect.h>
#include <behaviortree_mtc/bt/shared_to_unique.h>

#include <moveit/task_constructor/stages/connect.h>

namespace BT {
namespace MTC {

Connect::Connect(const std::string& name,
                 const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus Connect::tick()
{
  // Retrieve inputs
  double timeout;
  double max_distance;
  uint8_t merge_mode;
  std::string name, group;
  moveit::task_constructor::solvers::PlannerInterfacePtr planner;

  if(!getInput("timeout", timeout) ||
     !getInput("merge_mode", merge_mode) ||
     !getInput("max_distance", max_distance) ||
     !getInput("stage_name", name) ||
     !getInput("planner", planner) ||
     !getInput("group", group))
    return NodeStatus::FAILURE;

  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::Connect> stage{ nullptr };
  stage = {
    new moveit::task_constructor::stages::Connect(name, moveit::task_constructor::stages::Connect::GroupPlannerVector{ { group, planner } }),
    dirty::fake_deleter{}
  };
  stage->setTimeout(timeout);
  stage->setProperty("merge_mode", static_cast<moveit::task_constructor::stages::Connect::MergeMode>(merge_mode));
  stage->setProperty("max_distance", max_distance);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList Connect::providedPorts()
{
  return {
    //Output
    BT::OutputPort<moveit::task_constructor::StagePtr>("stage", "{connect_stage}", "Connect stage"),
    //Inputs
    BT::InputPort<double>("timeout", 1.0, "timeout"),
    BT::InputPort<double>("max_distance", 1e-4,
                          "maximally accepted joint configuration distance between trajectory endpoint and goal state"),
    BT::InputPort<std::string>("stage_name", "connect"),
    BT::InputPort<uint8_t>("merge_mode", 1, "0 = SEQUENTIAL, 1 = WAYPOINTS"),
    BT::InputPort<std::string>("group"),
    BT::InputPort<moveit::task_constructor::solvers::PlannerInterfacePtr>("planner"),
  };
}

}  // namespace MTC
}  // namespace BT
