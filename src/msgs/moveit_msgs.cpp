#include <behaviortree_mtc/msgs/moveit_msgs.h>
#include <behaviortree_mtc/serialization/custom_types.h>

#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>

namespace BT {
namespace MTC {

MoveItMsgsCollisionObjectBase::MoveItMsgsCollisionObjectBase(const std::string& name,
                                                             const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus MoveItMsgsCollisionObjectBase::tick(std::shared_ptr<moveit_msgs::CollisionObject> object)
{
  // Retrieve inputs
  std::string object_name, object_frame_id;
  auto pose = std::make_shared<geometry_msgs::Pose>();
  if(!getInput("object_id", object_name) ||
     !getInput("frame_id", object_frame_id) ||
     !getInput("pose", pose))
    return NodeStatus::FAILURE;

  // Build object
  object->id = object_name;
  object->header.frame_id = object_frame_id;
  object->primitive_poses.push_back(*pose);

  return NodeStatus::SUCCESS;
}

BT::PortsList MoveItMsgsCollisionObjectBase::providedPorts()
{
  return {
    BT::InputPort<std::string>("object_id"),
    BT::InputPort<std::string>("frame_id"),
    BT::InputPort<std::shared_ptr<geometry_msgs::Pose>>("pose")
  };
}

MoveItMsgsCollisionObjectCylinder::MoveItMsgsCollisionObjectCylinder(const std::string& name,
                                                                     const BT::NodeConfig& config)
  : MoveItMsgsCollisionObjectBase(name, config)
{}

BT::NodeStatus MoveItMsgsCollisionObjectCylinder::tick()
{
  // Build object
  auto object = std::make_shared<moveit_msgs::CollisionObject>();
  auto node_status = MoveItMsgsCollisionObjectBase::tick(object);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  // Retrieve inputs
  double radius, height;
  if(!getInput("height", height) ||
     !getInput("radius", radius))
    return NodeStatus::FAILURE;

  // Set dimensions
  object->primitives.resize(1);
  object->primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  object->primitives[0].dimensions = { height, radius };
  setOutput("collision_object", object);

  return NodeStatus::SUCCESS;
}

BT::PortsList MoveItMsgsCollisionObjectCylinder::providedPorts()
{
  auto port_lists = MoveItMsgsCollisionObjectBase::providedPorts();
  port_lists.emplace(BT::InputPort<double>("height", "1.0", "cylinder height (m)"));
  port_lists.emplace(BT::InputPort<double>("radius", "1.0", "cylinder radius (m)"));
  port_lists.emplace(BT::OutputPort<std::shared_ptr<moveit_msgs::CollisionObject>>("collision_object"));
  return port_lists;
}

MoveItMsgsCollisionObjectBox::MoveItMsgsCollisionObjectBox(const std::string& name,
                                                           const BT::NodeConfig& config)
  : MoveItMsgsCollisionObjectBase(name, config)
{}

BT::NodeStatus MoveItMsgsCollisionObjectBox::tick()
{
  // Build object
  auto object = std::make_shared<moveit_msgs::CollisionObject>();
  auto node_status = MoveItMsgsCollisionObjectBase::tick(object);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  // Retrieve inputs
  double length, height, width;
  if(!getInput("height", height) ||
     !getInput("length", length) ||
     !getInput("length", width))
    return NodeStatus::FAILURE;

  // Set dimensions
  object->primitives.resize(1);
  object->primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  object->primitives[0].dimensions = { length, width, height };
  setOutput("collision_object", object);

  return NodeStatus::SUCCESS;
}

BT::PortsList MoveItMsgsCollisionObjectBox::providedPorts()
{
  auto port_lists = MoveItMsgsCollisionObjectBase::providedPorts();
  port_lists.emplace(BT::InputPort<double>("length", "1.0", "Box's size along X axis (m)"));
  port_lists.emplace(BT::InputPort<double>("width", "1.0", "Box's size along Y axis (m)"));
  port_lists.emplace(BT::InputPort<double>("height", "1.0", "Box's size along Z axis (m)"));
  port_lists.emplace(BT::OutputPort<std::shared_ptr<moveit_msgs::CollisionObject>>("collision_object"));
  return port_lists;
}

}  // namespace MTC
}  // namespace BT
