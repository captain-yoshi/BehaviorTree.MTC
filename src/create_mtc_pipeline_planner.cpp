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
constexpr auto kPortGoalOrientationTolerance = "goal_orientation_tolerence";
constexpr auto kPortGoalPositionTolerance = "goal_position_tolerence";
constexpr auto kPortDisplayMotionPlans = "display_motion_plans";
constexpr auto kPortPublishPlanningRequests = "publish_planning_requests";
constexpr auto kPortNumPlanningAttempts = "num_planning_attempts";

}  // namespace

CreateMTCPipelinePlanner::CreateMTCPipelinePlanner(const std::string& name,
                                             const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}
BT::NodeStatus CreateMTCPipelinePlanner::tick()
{
  std::string pipeline_id;
  std::string planner_id;
  double goal_joint_tolerance;
  double goal_orientation_tolerance;
  double goal_position_tolerance;
  double max_velocity_scaling_factor;
  double max_acceleration_scaling_factor;
  bool display_motion_plans;
  bool publish_planning_requests;
  uint num_planning_attemps;
  //Mandatory inputs
  if(!getInput(kPortPipelineID, pipeline_id))  //ex : "ompl"
    return NodeStatus::FAILURE;
  if(!getInput(kPortPlannerID, planner_id))  //ex : "request_adapters"
    return NodeStatus::FAILURE;
  if(!getInput(kPortMaxVelocityScalingFactor, max_velocity_scaling_factor))
    return NodeStatus::FAILURE;
  if(!getInput(kPortMaxAccelerationScalingFactor, max_acceleration_scaling_factor))
    return NodeStatus::FAILURE;
  //Optionnal inputs
  getInput(kPortGoalJointTolerance, goal_joint_tolerance);
  getInput(kPortGoalPositionTolerance, goal_position_tolerance);
  getInput(kPortGoalOrientationTolerance, goal_orientation_tolerance);
  getInput(kPortDisplayMotionPlans, display_motion_plans);
  getInput(kPortPublishPlanningRequests, publish_planning_requests);
  getInput(kPortNumPlanningAttempts, num_planning_attemps);
  //build solver
  auto solver = std::make_shared<MTC::solvers::PipelinePlanner>(pipeline_id);
  solver->setPlannerId(planner_id);
  solver->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
  solver->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
  solver->setProperty(kPortGoalJointTolerance, goal_joint_tolerance);
  solver->setProperty(kPortGoalOrientationTolerance, goal_orientation_tolerance);
  solver->setProperty(kPortGoalPositionTolerance, goal_position_tolerance);
  solver->setProperty(kPortDisplayMotionPlans, display_motion_plans);
  solver->setProperty(kPortPublishPlanningRequests, publish_planning_requests);
  solver->setProperty(kPortNumPlanningAttempts, num_planning_attemps);
  // Upcast to base class
  MTC::solvers::PlannerInterfacePtr base_solver = solver;

  setOutput(kPortSolver, base_solver);
  return NodeStatus::SUCCESS;
};

BT::PortsList CreateMTCPipelinePlanner::providedPorts()
{
  return {
    //Output
    BT::OutputPort<MTC::solvers::PlannerInterfacePtr>(kPortSolver, "{solver}", "Planner interface using pipeline motion solver"),
    //Inputs
    BT::InputPort<double>(kPortGoalJointTolerance, "1e-4", "tolerance for reaching joint goals"),
    BT::InputPort<double>(kPortGoalPositionTolerance, "1e-4", "tolerance for reaching position goals"),
    BT::InputPort<double>(kPortGoalOrientationTolerance, "1e-4", "tolerance for reaching orientation goals"),
    BT::InputPort<bool>(kPortDisplayMotionPlans, false, "..."),
    BT::InputPort<bool>(kPortPublishPlanningRequests, false, "..."),
    BT::InputPort<std::string>(kPortPlannerID, "planner's id"),
    BT::InputPort<std::string>(kPortPipelineID, "pipeline's id"),
    BT::InputPort<uint>(kPortNumPlanningAttempts, "1u", "number of planning attempts"),
    BT::InputPort<double>(kPortMaxVelocityScalingFactor, "maximum velocity allowed"),
    BT::InputPort<double>(kPortMaxAccelerationScalingFactor, "maximum velocity allowed"),
  };
}
