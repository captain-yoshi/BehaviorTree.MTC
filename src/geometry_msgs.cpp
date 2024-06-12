#include <behaviortree_mtc/geometry_msgs.h>
#include <behaviortree_mtc/custom_types.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>

using namespace BT;
using namespace bt_mtc;

namespace
{
constexpr auto kPortFrameID = "frame_id";
constexpr auto kPortPosition = "position";
constexpr auto kPortVector = "vector";
constexpr auto kPortQuaternion = "quaternion";
constexpr auto kPortPoint = "point";
constexpr auto kPortPointStamped = "point_stamped";
constexpr auto kPortPoseStamped = "pose_stamped";
constexpr auto kPortVector3Stamped = "vector3_stamped";
constexpr auto kPortTwistStamped = "twist_stamped";
constexpr auto kPortLinearVelocity = "linear_velocity";
constexpr auto kPortAngularVelocity = "angular_velocity";
constexpr auto kPortPose = "pose";

}  // namespace
GeometryMsgsPointStamped::GeometryMsgsPointStamped(const std::string& name,
                                                   const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus GeometryMsgsPointStamped::tick()
{
  // Retrieve inputs
  std::string frame_id;
  Vector3D point;

  if(!getInput(kPortFrameID, frame_id) ||
     !getInput(kPortPoint, point))
    return NodeStatus::FAILURE;

  // Build pose
  auto point_stamped = std::make_shared<geometry_msgs::PointStamped>();

  point_stamped->header.frame_id = frame_id;
  point_stamped->point.x = point.x;
  point_stamped->point.y = point.y;
  point_stamped->point.z = point.z;

  setOutput(kPortPointStamped, point_stamped);

  return NodeStatus::SUCCESS;
}

BT::PortsList GeometryMsgsPointStamped::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortFrameID),
    BT::InputPort<Vector3D>(kPortPoint),
    BT::OutputPort<std::shared_ptr<geometry_msgs::PointStamped>>(kPortPointStamped),
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
  if(!getInput(kPortPosition, position) ||
     !getInput(kPortQuaternion, quaternion))
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

  setOutput(kPortPose, pose);

  return NodeStatus::SUCCESS;
}

BT::PortsList GeometryMsgsPose::providedPorts()
{
  return {
    BT::InputPort<Vector3D>(kPortPosition),
    BT::InputPort<Vector4D>(kPortQuaternion),
    BT::OutputPort<std::shared_ptr<geometry_msgs::Pose>>(kPortPose),
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

  if(!getInput(kPortFrameID, frame_id) ||
     !getInput(kPortPosition, position) ||
     !getInput(kPortQuaternion, quaternion))
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

  setOutput(kPortPoseStamped, pose);

  return NodeStatus::SUCCESS;
}

BT::PortsList GeometryMsgsPoseStamped::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortFrameID),
    BT::InputPort<Vector3D>(kPortPosition),
    BT::InputPort<Vector4D>(kPortQuaternion),
    BT::OutputPort<std::shared_ptr<geometry_msgs::PoseStamped>>(kPortPoseStamped),
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

  if(!getInput(kPortFrameID, frame_id) ||
     !getInput(kPortVector, vector))
    return NodeStatus::FAILURE;

  // Build pose
  auto v3 = std::make_shared<geometry_msgs::Vector3Stamped>();

  v3->header.frame_id = frame_id;
  v3->vector.x = vector.x;
  v3->vector.y = vector.y;
  v3->vector.z = vector.z;

  setOutput(kPortVector3Stamped, v3);

  return NodeStatus::SUCCESS;
}

BT::PortsList GeometryMsgsVector3Stamped::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortFrameID),
    BT::InputPort<Vector3D>(kPortVector),
    BT::OutputPort<std::shared_ptr<geometry_msgs::Vector3Stamped>>(kPortVector3Stamped),
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

  if(!getInput(kPortFrameID, frame_id) ||
     !getInput(kPortLinearVelocity, linear) ||
     !getInput(kPortAngularVelocity, angular))
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

  setOutput(kPortTwistStamped, twist);

  return NodeStatus::SUCCESS;
}

BT::PortsList GeometryMsgsTwistStamped::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortFrameID),
    BT::InputPort<Vector3D>(kPortLinearVelocity),
    BT::InputPort<Vector3D>(kPortAngularVelocity),
    BT::OutputPort<std::shared_ptr<geometry_msgs::TwistStamped>>(kPortTwistStamped),
  };
}
