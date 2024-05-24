#include <behaviortree_mtc/create_mtc_pipeline_planner.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

namespace
{
constexpr auto kPortSolver = "solver";
constexpr auto kPortPipelineID = "pipeline_id";
constexpr auto kPortPlannerID = "planner_id";
constexpr auto kPortGoalJointTolerance = "goal_joint_tolerance";
constexpr auto kPortMaxVelocityScalingFactor = "max_velocity_scaling_factor";
constexpr auto kPortMaxAccelerationScalingFactor = "max_acceleration_scaling_factor";

}  // namespace

CreateMTCPipelinePlanner::CreateMTCPipelinePlanner(const std::string& name,
                                                   const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreateMTCPipelinePlanner::tick()
{
  // Retrieve inputs
  std::string pipeline_id;
  std::string planner_id;
  double goal_joint_tolerance;
  double max_velocity_scaling_factor;
  double max_acceleration_scaling_factor;

  if(!getInput(kPortPipelineID, pipeline_id))
    return NodeStatus::FAILURE;
  if(!getInput(kPortPlannerID, planner_id))
    return NodeStatus::FAILURE;
  if(!getInput(kPortGoalJointTolerance, goal_joint_tolerance))
    return NodeStatus::FAILURE;
  if(!getInput(kPortMaxVelocityScalingFactor, max_velocity_scaling_factor))
    return NodeStatus::FAILURE;
  if(!getInput(kPortMaxAccelerationScalingFactor, max_acceleration_scaling_factor))
    return NodeStatus::FAILURE;

  // Build solver
  auto solver = std::make_shared<MTC::solvers::PipelinePlanner>(pipeline_id);
  solver->setPlannerId(planner_id);
  solver->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
  solver->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
  solver->setProperty(kPortGoalJointTolerance, goal_joint_tolerance);

  // Upcast to base class
  MTC::solvers::PlannerInterfacePtr base_solver = solver;

  setOutput(kPortSolver, base_solver);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCPipelinePlanner::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortPipelineID),
    BT::InputPort<std::string>(kPortPlannerID),
    BT::InputPort<double>(kPortGoalJointTolerance, 1e-5, ""),
    BT::InputPort<double>(kPortMaxVelocityScalingFactor),
    BT::InputPort<double>(kPortMaxAccelerationScalingFactor),
    BT::OutputPort<MTC::solvers::PlannerInterfacePtr>(kPortSolver),
  };
}
