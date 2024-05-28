#include <behaviortree_mtc/create_mtc_pipeline_planner.h>
<<<<<<< HEAD

#include <moveit/task_constructor/task.h>
=======
#include <behaviortree_mtc/shared_to_unique.h>

>>>>>>> [impl] pipeline planner solver
#include <moveit/task_constructor/solvers/pipeline_planner.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

<<<<<<< HEAD
CreatePipelinePlanner::CreatePipelinePlanner(const std::string& name,
                                             const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}
BT::NodeStatus CreatePipelinePlanner::tick()
{
  auto pipelinePlanner = std::make_shared<MTC::solvers::PipelinePlanner>();
  setPipelineProperties("goal_joint_tolerence", "double", pipelinePlanner);
  setPipelineProperties("goal_orientation_tolerence", "double", pipelinePlanner);
  setPipelineProperties("goal_position_tolerence", "double", pipelinePlanner);
  setPipelineProperties("display_motion_plans", "bool", pipelinePlanner);
  setPipelineProperties("publish_planning_requests", "bool", pipelinePlanner);
  setPipelineProperties("planner", "string", pipelinePlanner);
  setPipelineProperties("num_planning_attempts", "uint", pipelinePlanner);
  setPipelineProperties("max_velocity_scaling_factor", "double", pipelinePlanner);
  setPipelineProperties("max_acceleration_scaling_factor", "double", pipelinePlanner);
  setOutput("pipeline_planner", pipelinePlanner);
  return NodeStatus::SUCCESS;
};

void bt_mtc::CreatePipelinePlanner::setPipelineProperties(std::string property_name, std::string type, std::shared_ptr<MTC::solvers::PipelinePlanner> pipelinePlanner)
{
  if(type == "double")
  {
    double blackboard_input;
    getInput<double>(property_name, blackboard_input);
    pipelinePlanner->setProperty(property_name, blackboard_input);
  }
  if(type == "bool")
  {
    bool blackboard_input;
    getInput<bool>(property_name, blackboard_input);
    pipelinePlanner->setProperty(property_name, blackboard_input);
  }
  if(type == "string")
  {
    std::string blackboard_input;
    getInput<std::string>(property_name, blackboard_input);
    pipelinePlanner->setProperty(property_name, blackboard_input);
  }
  if(type == "uint")
  {
    uint blackboard_input;
    getInput<uint>(property_name, blackboard_input);
    pipelinePlanner->setProperty(property_name, blackboard_input);
  }
}

BT::PortsList CreatePipelinePlanner::providedPorts()
{
  return {
    BT::OutputPort<std::shared_ptr<MTC::solvers::PipelinePlanner>>("pipeline_planner", "Planner interface using pipeline motion solver"),
    BT::InputPort<double>("goal_joint_tolerence", "1e-4", "tolerance for reaching joint goals"),
    BT::InputPort<double>("goal_position_tolerence", "1e-4", "tolerance for reaching position goals"),
    BT::InputPort<double>("goal_orientation_tolerence", "1e-4", "tolerance for reaching orientation goals"),
    BT::InputPort<bool>("display_motion_plans", false, "..."),
    BT::InputPort<bool>("publish_planning_requests", false, "..."),
    BT::InputPort<std::string>("planner", "", "planner_id"),  //Verifier que c'est le bon default pour ompl et qu'on peut le changer a partir d'ici
    BT::InputPort<uint>("num_planning_attempts", "1u", "number of planning attempts"),
    BT::InputPort<double>("max_velocity_scaling_factor", "0.1", "maximum velocity allowed"),
    BT::InputPort<double>("max_acceleration_scaling_factor", "0.1", "maximum velocity allowed"),
    //BT::InputPort<double>( "timeout",,"time allowd for the planning task in ?s or ms"), a tester avant d'implenter
=======
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
>>>>>>> [impl] pipeline planner solver
  };
}
