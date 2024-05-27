#include <behaviortree_mtc/create_mtc_move_relative.h>
#include <behaviortree_mtc/shared_to_unique.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

using DirectionVariant = std::variant<std::shared_ptr<geometry_msgs::Vector3Stamped>,
                                      std::shared_ptr<std::map<std::string, double>>,
                                      std::shared_ptr<geometry_msgs::TwistStamped>>;

namespace
{
constexpr auto kPortStage = "stage";
constexpr auto kPortStageName = "stage_name";
constexpr auto kPortGroup = "group";
constexpr auto kPortSolver = "solver";
constexpr auto kPortIKFrame = "ik_frame";
constexpr auto kPortDirection = "direction";
constexpr auto kPortMinDistance = "min_distance";
constexpr auto kPortMaxDistance = "max_distance";
constexpr auto kPortTimeout = "timeout";

}  // namespace

CreateMTCMoveRelativeBase::CreateMTCMoveRelativeBase(const std::string& name,
                                                     const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreateMTCMoveRelativeBase::tick(std::shared_ptr<moveit::task_constructor::stages::MoveRelative>& stage)
{
  // Retrieve inputs
  std::string name;
  std::string group;
  MTC::solvers::PlannerInterfacePtr solver;
  std::shared_ptr<geometry_msgs::PoseStamped> ik_frame;
  double min_distance;
  double max_distance;
  double timeout;

  if(!getInput(kPortStageName, name) ||
     !getInput(kPortGroup, group) ||
     !getInput(kPortSolver, solver) ||
     !getInput(kPortIKFrame, ik_frame) ||
     !getInput(kPortMinDistance, min_distance) ||
     !getInput(kPortMaxDistance, max_distance) ||
     !getInput(kPortTimeout, timeout))
    return NodeStatus::FAILURE;

  // Build stage
  stage = {
    new MTC::stages::MoveRelative(name, solver),
    dirty::fake_deleter{}
  };

  stage->setGroup(group);
  stage->setIKFrame(*ik_frame);
  stage->setMinMaxDistance(min_distance, max_distance);
  stage->setTimeout(timeout);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveRelativeBase::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortStageName, "move relative", "stage name"),
    BT::InputPort<std::string>(kPortGroup, "name of planning group"),
    BT::InputPort<double>(kPortTimeout, 1.0, ""),
    BT::InputPort<double>(kPortMinDistance, -1.0, "minimum distance to move"),
    BT::InputPort<double>(kPortMaxDistance, 0.0, "maximum distance to move"),
    BT::InputPort<MTC::solvers::PlannerInterfacePtr>(kPortSolver, "planner interface"),
    BT::InputPort<std::shared_ptr<geometry_msgs::PoseStamped>>(kPortIKFrame, "frame to be moved in Cartesian direction"),
    BT::OutputPort<MTC::StagePtr>(kPortStage, "MoveRelative stage"),
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
  if(!getInput(kPortDirection, twist))
    return NodeStatus::FAILURE;

  stage->setDirection(*twist);

  // Upcast to base class
  MTC::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveRelativeTwist::providedPorts()
{
  auto port_lists = CreateMTCMoveRelativeBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<geometry_msgs::TwistStamped>>(kPortDirection, "perform twist motion on specified link"));

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
  if(!getInput(kPortDirection, vector3))
    return NodeStatus::FAILURE;

  stage->setDirection(*vector3);

  // Upcast to base class
  MTC::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveRelativeTranslate::providedPorts()
{
  auto port_lists = CreateMTCMoveRelativeBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<geometry_msgs::Vector3Stamped>>(kPortDirection, "translate link along given direction"));

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
  if(!getInput(kPortDirection, joint_deltas))
    return NodeStatus::FAILURE;

  stage->setDirection(*joint_deltas);

  // Upcast to base class
  MTC::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveRelativeJoint::providedPorts()
{
  auto port_lists = CreateMTCMoveRelativeBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<std::map<std::string, double>>>(kPortDirection, "move specified joint variables by given amount"));

  return port_lists;
}
