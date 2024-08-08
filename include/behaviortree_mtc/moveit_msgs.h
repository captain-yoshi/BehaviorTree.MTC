#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_msgs/CollisionObject.h>

namespace BT {
namespace MTC {

class MoveItMsgsCollisionObjectBase : public BT::SyncActionNode
{
public:
  MoveItMsgsCollisionObjectBase(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick(std::shared_ptr<moveit_msgs::CollisionObject> object);
  static BT::PortsList providedPorts();
};
class MoveItMsgsCollisionObjectCylinder : public MoveItMsgsCollisionObjectBase
{
public:
  MoveItMsgsCollisionObjectCylinder(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};
class MoveItMsgsCollisionObjectBox : public MoveItMsgsCollisionObjectBase
{
public:
  MoveItMsgsCollisionObjectBox(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

}  // namespace MTC
}  // namespace BT
