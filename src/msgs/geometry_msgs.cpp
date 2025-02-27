#include <behaviortree_mtc/msgs/geometry_msgs.h>
#include <behaviortree_mtc/serialization/custom_types.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>

namespace BT {
namespace MTC {

GeometryMsgsPointStamped::GeometryMsgsPointStamped(const std::string& name,
                                                   const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus GeometryMsgsPointStamped::tick()
{
  // Retrieve inputs
  std::string frame_id;
  Vector3D point;

  if(!getInput("frame_id", frame_id) ||
     !getInput("point", point))
    return NodeStatus::FAILURE;

  // Build pose
  auto point_stamped = std::make_shared<geometry_msgs::PointStamped>();

  point_stamped->header.frame_id = frame_id;
  point_stamped->point.x = point.x;
  point_stamped->point.y = point.y;
  point_stamped->point.z = point.z;

  setOutput("point_stamped", point_stamped);

  return NodeStatus::SUCCESS;
}

BT::PortsList GeometryMsgsPointStamped::providedPorts()
{
  return {
    BT::InputPort<std::string>("frame_id"),
    BT::InputPort<Vector3D>("point"),
    BT::OutputPort<std::shared_ptr<geometry_msgs::PointStamped>>("point_stamped"),
  };
}

GeometryMsgsPose::GeometryMsgsPose(const std::string& name,
                                   const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus GeometryMsgsPose::tick()
{
  // Retrieve inputs
  Vector3D position;
  Vector4D quaternion;
  if(!getInput("position", position) ||
     !getInput("quaternion", quaternion))
    return NodeStatus::FAILURE;

  // Build pose
  auto pose = std::make_shared<geometry_msgs::Pose>();

  pose->position.x = position.x;
  pose->position.y = position.y;
  pose->position.z = position.z;

  pose->orientation.w = quaternion.w;
  pose->orientation.x = quaternion.x;
  pose->orientation.y = quaternion.y;
  pose->orientation.z = quaternion.z;

  setOutput("pose", pose);

  return NodeStatus::SUCCESS;
}

BT::PortsList GeometryMsgsPose::providedPorts()
{
  return {
    BT::InputPort<Vector3D>("position"),
    BT::InputPort<Vector4D>("quaternion"),
    BT::OutputPort<std::shared_ptr<geometry_msgs::Pose>>("pose"),
  };
}

GeometryMsgsPoseStamped::GeometryMsgsPoseStamped(const std::string& name,
                                                 const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus GeometryMsgsPoseStamped::tick()
{
  // Retrieve inputs
  std::string frame_id;
  Vector3D position;
  Vector4D quaternion;

  if(!getInput("frame_id", frame_id) ||
     !getInput("position", position) ||
     !getInput("quaternion", quaternion))
    return NodeStatus::FAILURE;

  // Build pose
  auto pose = std::make_shared<geometry_msgs::PoseStamped>();

  pose->header.frame_id = frame_id;
  pose->pose.position.x = position.x;
  pose->pose.position.y = position.y;
  pose->pose.position.z = position.z;

  pose->pose.orientation.w = quaternion.w;
  pose->pose.orientation.x = quaternion.x;
  pose->pose.orientation.y = quaternion.y;
  pose->pose.orientation.z = quaternion.z;

  setOutput("pose_stamped", pose);

  return NodeStatus::SUCCESS;
}

BT::PortsList GeometryMsgsPoseStamped::providedPorts()
{
  return {
    BT::InputPort<std::string>("frame_id"),
    BT::InputPort<Vector3D>("position"),
    BT::InputPort<Vector4D>("quaternion"),
    BT::OutputPort<std::shared_ptr<geometry_msgs::PoseStamped>>("pose_stamped"),
  };
}

GeometryMsgsVector3Stamped::GeometryMsgsVector3Stamped(const std::string& name,
                                                       const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus GeometryMsgsVector3Stamped::tick()
{
  // Retrieve inputs
  std::string frame_id;
  Vector3D vector;

  if(!getInput("frame_id", frame_id) ||
     !getInput("vector", vector))
    return NodeStatus::FAILURE;

  // Build pose
  auto v3 = std::make_shared<geometry_msgs::Vector3Stamped>();

  v3->header.frame_id = frame_id;
  v3->vector.x = vector.x;
  v3->vector.y = vector.y;
  v3->vector.z = vector.z;

  setOutput("vector3_stamped", v3);

  return NodeStatus::SUCCESS;
}

BT::PortsList GeometryMsgsVector3Stamped::providedPorts()
{
  return {
    BT::InputPort<std::string>("frame_id"),
    BT::InputPort<Vector3D>("vector"),
    BT::OutputPort<std::shared_ptr<geometry_msgs::Vector3Stamped>>("vector3_stamped"),
  };
}

GeometryMsgsTwistStamped::GeometryMsgsTwistStamped(const std::string& name,
                                                   const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus GeometryMsgsTwistStamped::tick()
{
  // Retrieve inputs
  std::string frame_id;
  Vector3D linear;
  Vector3D angular;

  if(!getInput("frame_id", frame_id) ||
     !getInput("linear_velocity", linear) ||
     !getInput("angular_velocity", angular))
    return NodeStatus::FAILURE;

  // Build pose
  auto twist = std::make_shared<geometry_msgs::TwistStamped>();

  twist->header.frame_id = frame_id;
  twist->twist.linear.x = linear.x;
  twist->twist.linear.y = linear.y;
  twist->twist.linear.z = linear.z;

  twist->twist.angular.x = angular.x;
  twist->twist.angular.y = angular.y;
  twist->twist.angular.z = angular.z;

  setOutput("twist_stamped", twist);

  return NodeStatus::SUCCESS;
}

BT::PortsList GeometryMsgsTwistStamped::providedPorts()
{
  return {
    BT::InputPort<std::string>("frame_id"),
    BT::InputPort<Vector3D>("linear_velocity"),
    BT::InputPort<Vector3D>("angular_velocity"),
    BT::OutputPort<std::shared_ptr<geometry_msgs::TwistStamped>>("twist_stamped"),
  };
}

}  // namespace MTC
}  // namespace BT
