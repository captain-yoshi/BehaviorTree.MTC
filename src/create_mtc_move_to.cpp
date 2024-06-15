#include <behaviortree_mtc/create_mtc_move_to.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <behaviortree_mtc/std_containers.h>

namespace BT {
namespace MTC {

namespace {
constexpr auto kPortStageName = "stage_name";
constexpr auto kPortStage = "stage";
constexpr auto kPortSolver = "solver";
constexpr auto kPortGroup = "group";
constexpr auto kPortTimeout = "timeout";
constexpr auto kPortIKFrame = "ik_frame";
constexpr auto kPortGoal = "goal";

}  // namespace

CreateMTCMoveToBase::CreateMTCMoveToBase(const std::string& name,
                                         const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreateMTCMoveToBase::tick(std::shared_ptr<moveit::task_constructor::stages::MoveTo>& stage)
{
  // Retrieve inputs
  std::string name;
  std::string group;
  moveit::task_constructor::solvers::PlannerInterfacePtr solver;
  double min_distance;
  double max_distance;
  double timeout;

  if(!getInput(kPortStageName, name) ||
     !getInput(kPortGroup, group) ||
     !getInput(kPortSolver, solver) ||
     !getInput(kPortTimeout, timeout))
    return NodeStatus::FAILURE;

  // Build stage
  stage = {
    new moveit::task_constructor::stages::MoveTo(name, solver),
    dirty::fake_deleter{}
  };

  stage->setGroup(group);
  stage->setTimeout(timeout);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveToBase::providedPorts()
{
  return {
    BT::OutputPort<moveit::task_constructor::StagePtr>(kPortStage, "MoveTo stage"),
    BT::InputPort<std::string>(kPortStageName, "move to", "stage name"),
    BT::InputPort<std::string>(kPortGroup, "name of planning group"),
    BT::InputPort<double>(kPortTimeout, 1.0, ""),
    BT::InputPort<moveit::task_constructor::solvers::PlannerInterfacePtr>(kPortSolver, "planner interface"),
  };
}

CreateMTCMoveToBaseCartesian::CreateMTCMoveToBaseCartesian(const std::string& name,
                                                           const BT::NodeConfig& config)
  : CreateMTCMoveToBase(name, config)
{}

BT::NodeStatus CreateMTCMoveToBaseCartesian::tick(std::shared_ptr<moveit::task_constructor::stages::MoveTo>& stage)
{
  auto node_status = CreateMTCMoveToBase::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  std::shared_ptr<geometry_msgs::PoseStamped> ik_frame;
  if(!getInput(kPortIKFrame, ik_frame))
    return NodeStatus::FAILURE;
  stage->setIKFrame(*ik_frame);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveToBaseCartesian::providedPorts()
{
  auto port_lists = CreateMTCMoveToBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<geometry_msgs::PoseStamped>>(kPortIKFrame, "frame to be moved in Cartesian direction"));
  return port_lists;
}

CreateMTCMoveToNamedJointPose::CreateMTCMoveToNamedJointPose(const std::string& name,
                                                             const BT::NodeConfig& config)
  : CreateMTCMoveToBase(name, config)
{}

BT::NodeStatus CreateMTCMoveToNamedJointPose::tick()
{
  //Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveTo> stage{ nullptr };
  auto node_status = CreateMTCMoveToBase::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  //Set goal
  std::string named_joint_pose;
  if(!getInput(kPortGoal, named_joint_pose))
    return NodeStatus::FAILURE;
  stage->setGoal(named_joint_pose);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;
  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveToNamedJointPose::providedPorts()
{
  auto port_lists = CreateMTCMoveToBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::string>(kPortGoal, "move joint model group to given named pose"));

  return port_lists;
}

CreateMTCMoveToJoint::CreateMTCMoveToJoint(const std::string& name,
                                           const BT::NodeConfig& config)
  : CreateMTCMoveToBase(name, config)
{}

BT::NodeStatus CreateMTCMoveToJoint::tick()
{
  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveTo> stage{ nullptr };
  auto node_status = CreateMTCMoveToBase::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  // Set goal
  std::shared_ptr<std::map<std::string, double>> joints{ nullptr };
  if(!getInput(kPortGoal, joints))
    return NodeStatus::FAILURE;

  stage->setGoal(*joints);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveToJoint::providedPorts()
{
  auto port_lists = CreateMTCMoveToBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<std::map<std::string, double>>>(kPortGoal, "move joints by name to their mapped target value"));

  return port_lists;
}

CreateMTCMoveToPose::CreateMTCMoveToPose(const std::string& name,
                                         const BT::NodeConfig& config)
  : CreateMTCMoveToBaseCartesian(name, config)
{}

BT::NodeStatus CreateMTCMoveToPose::tick()
{
  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveTo> stage{ nullptr };
  auto node_status = CreateMTCMoveToBaseCartesian::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  // Set goal
  std::shared_ptr<geometry_msgs::PoseStamped> pose{ nullptr };
  if(!getInput(kPortGoal, pose))
    return NodeStatus::FAILURE;

  stage->setGoal(*pose);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveToPose::providedPorts()
{
  auto port_lists = CreateMTCMoveToBaseCartesian::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<geometry_msgs::PoseStamped>>(kPortGoal, "move link to a given pose"));

  return port_lists;
}

CreateMTCMoveToPoint::CreateMTCMoveToPoint(const std::string& name,
                                           const BT::NodeConfig& config)
  : CreateMTCMoveToBaseCartesian(name, config)
{}

BT::NodeStatus CreateMTCMoveToPoint::tick()
{
  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveTo> stage{ nullptr };
  auto node_status = CreateMTCMoveToBaseCartesian::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  // Set goal
  std::shared_ptr<geometry_msgs::PointStamped> point{ nullptr };
  if(!getInput(kPortGoal, point))
    return NodeStatus::FAILURE;

  stage->setGoal(*point);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveToPoint::providedPorts()
{
  auto port_lists = CreateMTCMoveToBaseCartesian::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<geometry_msgs::PointStamped>>(kPortGoal, "move link to given point, keeping current orientation"));

  return port_lists;
}

}  // namespace MTC
}  // namespace BT
