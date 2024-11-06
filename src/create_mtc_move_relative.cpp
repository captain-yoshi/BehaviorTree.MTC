#include <behaviortree_mtc/create_mtc_move_relative.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <behaviortree_mtc/std_containers.h>

namespace BT {
namespace MTC {

CreateMTCMoveRelativeBase::CreateMTCMoveRelativeBase(const std::string& name,
                                                     const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreateMTCMoveRelativeBase::tick(std::shared_ptr<moveit::task_constructor::stages::MoveRelative>& stage)
{
  // Retrieve inputs
  std::string name;
  std::string group;
  moveit::task_constructor::solvers::PlannerInterfacePtr solver;
  std::shared_ptr<geometry_msgs::PoseStamped> ik_frame;
  double min_distance;
  double max_distance;
  double timeout;

  if(!getInput("stage_name", name) ||
     !getInput("solver", solver) ||
     !getInput("ik_frame", ik_frame) ||
     !getInput("min_distance", min_distance) ||
     !getInput("max_distance", max_distance) ||
     !getInput("timeout", timeout))
    return NodeStatus::FAILURE;

  // Build stage
  stage = {
    new moveit::task_constructor::stages::MoveRelative(name, solver),
    dirty::fake_deleter{}
  };

  //Optionnal
  if(getInput("group", group))
    stage->setGroup(group);

  stage->setIKFrame(*ik_frame);
  stage->setMinMaxDistance(min_distance, max_distance);
  stage->setTimeout(timeout);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveRelativeBase::providedPorts()
{
  return {
    BT::InputPort<std::string>("stage_name", "move relative", "stage name"),
    BT::InputPort<std::string>("group", "name of planning group"),
    BT::InputPort<double>("timeout", 1.0, ""),
    BT::InputPort<double>("min_distance", -1.0, "minimum distance to move"),
    BT::InputPort<double>("max_distance", 0.0, "maximum distance to move"),
    BT::InputPort<moveit::task_constructor::solvers::PlannerInterfacePtr>("solver", "planner interface"),
    BT::InputPort<std::shared_ptr<geometry_msgs::PoseStamped>>("ik_frame", "frame to be moved in Cartesian direction"),
    BT::OutputPort<moveit::task_constructor::StagePtr>("stage", "MoveRelative stage"),
  };
}

CreateMTCMoveRelativeTwist::CreateMTCMoveRelativeTwist(const std::string& name,
                                                       const BT::NodeConfig& config)
  : CreateMTCMoveRelativeBase(name, config)
{}

BT::NodeStatus CreateMTCMoveRelativeTwist::tick()
{
  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveRelative> stage{ nullptr };
  auto node_status = CreateMTCMoveRelativeBase::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  // Set direction
  std::shared_ptr<geometry_msgs::TwistStamped> twist{ nullptr };
  if(!getInput("direction", twist))
    return NodeStatus::FAILURE;

  stage->setDirection(*twist);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveRelativeTwist::providedPorts()
{
  auto port_lists = CreateMTCMoveRelativeBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<geometry_msgs::TwistStamped>>("direction", "perform twist motion on specified link"));

  return port_lists;
}

CreateMTCMoveRelativeTranslate::CreateMTCMoveRelativeTranslate(const std::string& name,
                                                               const BT::NodeConfig& config)
  : CreateMTCMoveRelativeBase(name, config)
{}

BT::NodeStatus CreateMTCMoveRelativeTranslate::tick()
{
  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveRelative> stage{ nullptr };
  auto node_status = CreateMTCMoveRelativeBase::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  // Set direction
  std::shared_ptr<geometry_msgs::Vector3Stamped> vector3{ nullptr };
  if(!getInput("direction", vector3))
    return NodeStatus::FAILURE;

  stage->setDirection(*vector3);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveRelativeTranslate::providedPorts()
{
  auto port_lists = CreateMTCMoveRelativeBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<geometry_msgs::Vector3Stamped>>("direction", "translate link along given direction"));

  return port_lists;
}

CreateMTCMoveRelativeJoint::CreateMTCMoveRelativeJoint(const std::string& name,
                                                       const BT::NodeConfig& config)
  : CreateMTCMoveRelativeBase(name, config)
{}

BT::NodeStatus CreateMTCMoveRelativeJoint::tick()
{
  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveRelative> stage{ nullptr };
  auto node_status = CreateMTCMoveRelativeBase::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  // Set direction
  std::shared_ptr<std::map<std::string, double>> joint_deltas{ nullptr };
  if(!getInput("direction", joint_deltas))
    return NodeStatus::FAILURE;

  stage->setDirection(*joint_deltas);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveRelativeJoint::providedPorts()
{
  auto port_lists = CreateMTCMoveRelativeBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<std::map<std::string, double>>>("direction", "move specified joint variables by given amount"));

  return port_lists;
}

}  // namespace MTC
}  // namespace BT
