#include <behaviortree_mtc/create_mtc_pipeline_planner.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <rclcpp/node.hpp>

namespace BT {
namespace MTC {

CreateMTCPipelinePlanner::CreateMTCPipelinePlanner(const std::string& name,
                                                   const BT::NodeConfig& config, 
                                                   const BT::RosNodeParams& params)
  : SyncActionNode(name, config), node_(params.nh.lock())
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

  //Inputs
  if(!getInput("pipeline_id", pipeline_id) ||
     !getInput("planner_id", planner_id) ||
     !getInput("max_velocity_scaling_factor", max_velocity_scaling_factor) ||
     !getInput("max_acceleration_scaling_factor", max_acceleration_scaling_factor) ||
     !getInput("goal_joint_tolerance", goal_joint_tolerance) ||
     !getInput("goal_position_tolerance", goal_position_tolerance) ||
     !getInput("goal_orientation_tolerance", goal_orientation_tolerance) ||
     !getInput("display_motion_plans", display_motion_plans) ||
     !getInput("publish_planning_requests", publish_planning_requests) ||
     !getInput("num_planning_attempts", num_planning_attemps))
    return NodeStatus::FAILURE;
  //build solver
  auto solver = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_, pipeline_id);
  solver->setPlannerId(planner_id);
  solver->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
  solver->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
  solver->setProperty("goal_joint_tolerance", goal_joint_tolerance);
  solver->setProperty("goal_orientation_tolerance", goal_orientation_tolerance);
  solver->setProperty("goal_position_tolerance", goal_position_tolerance);
  solver->setProperty("display_motion_plans", display_motion_plans);
  solver->setProperty("publish_planning_requests", publish_planning_requests);
  solver->setProperty("num_planning_attempts", num_planning_attemps);
  // Upcast to base class
  moveit::task_constructor::solvers::PlannerInterfacePtr base_solver = solver;

  setOutput("solver", base_solver);
  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCPipelinePlanner::providedPorts()
{
  return {
    //Output
    BT::OutputPort<moveit::task_constructor::solvers::PlannerInterfacePtr>("solver", "{solver}", "Planner interface using pipeline motion solver"),
    //Inputs
    BT::InputPort<double>("goal_joint_tolerance", "1e-4", "tolerance for reaching joint goals"),
    BT::InputPort<double>("goal_position_tolerance", "1e-4", "tolerance for reaching position goals"),
    BT::InputPort<double>("goal_orientation_tolerance", "1e-4", "tolerance for reaching orientation goals"),
    BT::InputPort<bool>("display_motion_plans", false, "publish generated solutions on topic " + planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC),
    BT::InputPort<bool>("publish_planning_requests", false, "publish motion planning requests on topic " + planning_pipeline::PlanningPipeline::MOTION_PLAN_REQUEST_TOPIC),
    BT::InputPort<std::string>("planner_id"),
    BT::InputPort<std::string>("pipeline_id"),
    BT::InputPort<uint>("num_planning_attempts", "1u", "number of planning attempts"),
    BT::InputPort<double>("max_velocity_scaling_factor", 0.1, "scale down max velocity by this factor"),
    BT::InputPort<double>("max_acceleration_scaling_factor", 0.1, "scale down max acceleration by this factor"),
  };
}

}  // namespace MTC
}  // namespace BT
