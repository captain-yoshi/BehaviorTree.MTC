#include <behaviortree_mtc/create_mtc_pipeline_planner.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <rclcpp/node.hpp>

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
  // Retrieve inputs
  std::string pipeline_id;
  std::string planner_id;
  double goal_joint_tolerance;
  double max_velocity_scaling_factor;
  double max_acceleration_scaling_factor;
  double goal_orientation_tolerance;
  double goal_position_tolerance;
  bool display_motion_plans;
  bool publish_planning_requests;
  uint num_planning_attemps;

  if (!getInput(kPortPipelineID, pipeline_id))
    return NodeStatus::FAILURE;
  if (!getInput(kPortPlannerID, planner_id))
    return NodeStatus::FAILURE;
  if (!getInput(kPortMaxVelocityScalingFactor, max_velocity_scaling_factor))
    return NodeStatus::FAILURE;
  if (!getInput(kPortMaxAccelerationScalingFactor, max_acceleration_scaling_factor))
    return NodeStatus::FAILURE;
  getInput(kPortGoalJointTolerance, goal_joint_tolerance);  // optional

  // Build solver
  auto node = rclcpp::Node::make_shared("create_mtc_pipeline_planner");
  auto solver = std::make_shared<MTC::solvers::PipelinePlanner>(node, pipeline_id);

  // Set up the solver with the retrieved inputs
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
}

BT::PortsList CreateMTCPipelinePlanner::providedPorts()
{
  return {
    //Output
    BT::OutputPort<MTC::solvers::PlannerInterfacePtr>(kPortSolver, "{solver}", "Planner interface using pipeline motion solver"),
    //Inputs
    BT::InputPort<double>(kPortGoalJointTolerance, "1e-4", "tolerance for reaching joint goals"),
    BT::InputPort<double>(kPortGoalPositionTolerance, "1e-4", "tolerance for reaching position goals"),
    BT::InputPort<double>(kPortGoalOrientationTolerance, "1e-4", "tolerance for reaching orientation goals"),
    BT::InputPort<bool>(kPortDisplayMotionPlans, false, "publish generated solutions on topic " + planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC),
    BT::InputPort<bool>(kPortPublishPlanningRequests, false, "publish motion planning requests on topic " + planning_pipeline::PlanningPipeline::MOTION_PLAN_REQUEST_TOPIC),
    BT::InputPort<std::string>(kPortPlannerID),
    BT::InputPort<std::string>(kPortPipelineID),
    BT::InputPort<uint>(kPortNumPlanningAttempts, "1u", "number of planning attempts"),
    BT::InputPort<double>(kPortMaxVelocityScalingFactor, 0.1, "scale down max velocity by this factor"),
    BT::InputPort<double>(kPortMaxAccelerationScalingFactor, 0.1, "scale down max acceleration by this factor"),
  };
}
