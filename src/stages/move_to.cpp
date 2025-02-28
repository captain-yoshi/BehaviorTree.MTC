#include <behaviortree_mtc/stages/move_to.h>
#include <behaviortree_mtc/bt/shared_to_unique.h>

#include <behaviortree_mtc/serialization/std_containers.h>

namespace BT {
namespace MTC {

MoveToBase::MoveToBase(const std::string& name,
                       const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus MoveToBase::tick(std::shared_ptr<moveit::task_constructor::stages::MoveTo>& stage)
{
  // Retrieve inputs
  std::string name;
  std::string group;
  moveit::task_constructor::solvers::PlannerInterfacePtr solver;
  double min_distance;
  double max_distance;
  double timeout;

  if(!getInput("stage_name", name) ||
     !getInput("solver", solver) ||
     !getInput("timeout", timeout))
    return NodeStatus::FAILURE;

  // Build stage
  stage = {
    new moveit::task_constructor::stages::MoveTo(name, solver),
    dirty::fake_deleter{}
  };
  // Optionnal
  if(getInput("group", group))
    stage->setGroup(group);

  stage->setTimeout(timeout);

  return NodeStatus::SUCCESS;
}

BT::PortsList MoveToBase::providedPorts()
{
  return {
    BT::InputPort<std::string>("stage_name", "move to", "stage name"),
    BT::InputPort<std::string>("group", "name of planning group"),
    BT::InputPort<double>("timeout", 1.0, ""),
    BT::InputPort<moveit::task_constructor::solvers::PlannerInterfacePtr>("solver", "planner interface"),
    BT::OutputPort<moveit::task_constructor::StagePtr>("stage", "MoveTo stage"),
  };
}

MoveToBaseCartesian::MoveToBaseCartesian(const std::string& name,
                                         const BT::NodeConfig& config)
  : MoveToBase(name, config)
{}

BT::NodeStatus MoveToBaseCartesian::tick(std::shared_ptr<moveit::task_constructor::stages::MoveTo>& stage)
{
  auto node_status = MoveToBase::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  std::shared_ptr<geometry_msgs::PoseStamped> ik_frame;
  if(!getInput("ik_frame", ik_frame))
    return NodeStatus::FAILURE;
  stage->setIKFrame(*ik_frame);

  return NodeStatus::SUCCESS;
}

BT::PortsList MoveToBaseCartesian::providedPorts()
{
  auto port_lists = MoveToBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<geometry_msgs::PoseStamped>>("ik_frame", "frame to be moved in Cartesian direction"));
  return port_lists;
}

MoveToNamedJointPose::MoveToNamedJointPose(const std::string& name,
                                           const BT::NodeConfig& config)
  : MoveToBase(name, config)
{}

BT::NodeStatus MoveToNamedJointPose::tick()
{
  //Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveTo> stage{ nullptr };
  auto node_status = MoveToBase::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  //Set goal
  std::string named_joint_pose;
  if(!getInput("goal", named_joint_pose))
    return NodeStatus::FAILURE;
  stage->setGoal(named_joint_pose);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;
  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList MoveToNamedJointPose::providedPorts()
{
  auto port_lists = MoveToBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::string>("goal", "move joint model group to given named pose"));

  return port_lists;
}

MoveToJoint::MoveToJoint(const std::string& name,
                         const BT::NodeConfig& config)
  : MoveToBase(name, config)
{}

BT::NodeStatus MoveToJoint::tick()
{
  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveTo> stage{ nullptr };
  auto node_status = MoveToBase::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  // Set goal
  std::shared_ptr<std::map<std::string, double>> joints{ nullptr };
  if(!getInput("goal", joints))
    return NodeStatus::FAILURE;

  stage->setGoal(*joints);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList MoveToJoint::providedPorts()
{
  auto port_lists = MoveToBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<std::map<std::string, double>>>("goal", "move joints by name to their mapped target value"));

  return port_lists;
}

MoveToPose::MoveToPose(const std::string& name,
                       const BT::NodeConfig& config)
  : MoveToBaseCartesian(name, config)
{}

BT::NodeStatus MoveToPose::tick()
{
  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveTo> stage{ nullptr };
  auto node_status = MoveToBaseCartesian::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  // Set goal
  std::shared_ptr<geometry_msgs::PoseStamped> pose{ nullptr };
  if(!getInput("goal", pose))
    return NodeStatus::FAILURE;

  stage->setGoal(*pose);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList MoveToPose::providedPorts()
{
  auto port_lists = MoveToBaseCartesian::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<geometry_msgs::PoseStamped>>("goal", "move link to a given pose"));

  return port_lists;
}

MoveToPoint::MoveToPoint(const std::string& name,
                         const BT::NodeConfig& config)
  : MoveToBaseCartesian(name, config)
{}

BT::NodeStatus MoveToPoint::tick()
{
  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveTo> stage{ nullptr };
  auto node_status = MoveToBaseCartesian::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  // Set goal
  std::shared_ptr<geometry_msgs::PointStamped> point{ nullptr };
  if(!getInput("goal", point))
    return NodeStatus::FAILURE;

  stage->setGoal(*point);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList MoveToPoint::providedPorts()
{
  auto port_lists = MoveToBaseCartesian::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<geometry_msgs::PointStamped>>("goal", "move link to given point, keeping current orientation"));

  return port_lists;
}

}  // namespace MTC
}  // namespace BT
