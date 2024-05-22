#include <behaviortree_mtc/create_mtc_pipeline_planner.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

using namespace BT;
namespace MTC = moveit::task_constructor;
using namespace bt_mtc;

// namespace
// {
// constexpr auto kPortPlanner = "sampling_planner";

// }  // namespace

CreatePipelinePlanner::CreatePipelinePlanner(const std::string& name,
                                     const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreatePipelinePlanner::tick()
{
  if(auto any_ptr = getLockedPortContent("pipeline_planner")) // changer sa pour la bonne chose
  {
    printf("initializing a pipeline planner");
    auto sampling_planner = std::make_shared<MTC::solvers::PipelinePlanner>();
    auto tolerence = getInput<float>("goal_joint_tolerence");
    sampling_planner->setProperty("goal_joint_tolerance", tolerence); //remplacer la valeur par un input
    any_ptr.assign(sampling_planner);
    printf("pipeline planner initialized");
    return NodeStatus::SUCCESS;
  }
  else
  {
    return NodeStatus::FAILURE;
  }
}

BT::PortsList CreatePipelinePlanner::providedPorts()
{
  const char* description_input = "goal joint tolerence";
  const char* description_output = "MoveIt Task Constructor task.";
  
  return {
    BT::OutputPort<std::shared_ptr<MTC::solvers::PipelinePlanner>>("pipeline_planner", description_output),
    BT::InputPort<float>("goal_joint_tolerance",1e-5, description_input), 
  };
}
//INPUTS : 
//-Pipeline name (needed in the constructor)
//-goal_joint_tolerance
//-goal_position_tolerance
//-orientation_tolerence
//"planner" ->planner_id
//"num_planning_attempts" 
//"publish_planning_requests" bool
//"display_motion_plans" bool
//va falloir surment pointer au robot model pour la fonction init...model est jamais caller... 
//modelvelocity appartienne au planning interface