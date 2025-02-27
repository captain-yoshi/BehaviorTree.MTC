#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace BT {
namespace MTC {

/// Base class
class AttachDetachObjects : public BT::SyncActionNode
{
public:
  AttachDetachObjects(const std::string& name, const BT::NodeConfig& config, bool attach);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  const bool _attach;  // true = attach, false = detach
};

class AllowForbidCollisions : public BT::SyncActionNode
{
public:
  AllowForbidCollisions(const std::string& name, const BT::NodeConfig& config, bool allow);

  BT::NodeStatus tick(const std::vector<std::string>& first, const std::vector<std::string>& second);
  static BT::PortsList providedPorts();

private:
  const bool _allow;  // true = allow collisions, false = forbid collisions
};

class AllowForbidCollisionPairs : public AllowForbidCollisions
{
public:
  AllowForbidCollisionPairs(const std::string& name, const BT::NodeConfig& config, bool allow);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class AllowForbidAllCollisions : public AllowForbidCollisions
{
public:
  AllowForbidAllCollisions(const std::string& name, const BT::NodeConfig& config, bool allow);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class AllowForbidCollisionsJointModelGroup : public AllowForbidCollisions
{
public:
  AllowForbidCollisionsJointModelGroup(const std::string& name, const BT::NodeConfig& config, bool allow);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

/// Actual BT nodes
class CreateMTCModifyPlanningSceneAttachObjects : public AttachDetachObjects
{
public:
  CreateMTCModifyPlanningSceneAttachObjects(const std::string& name, const BT::NodeConfig& config)
    : AttachDetachObjects(name, config, true)
  {}
};

class CreateMTCModifyPlanningSceneDetachObjects : public AttachDetachObjects
{
public:
  CreateMTCModifyPlanningSceneDetachObjects(const std::string& name, const BT::NodeConfig& config)
    : AttachDetachObjects(name, config, false)
  {}
};

class AllowCollisionPairs : public AllowForbidCollisionPairs
{
public:
  AllowCollisionPairs(const std::string& name, const BT::NodeConfig& config)
    : AllowForbidCollisionPairs(name, config, true)
  {}
};

class ForbidCollisionPairs : public AllowForbidCollisionPairs
{
public:
  ForbidCollisionPairs(const std::string& name, const BT::NodeConfig& config)
    : AllowForbidCollisionPairs(name, config, false)
  {}
};

class AllowAllCollisions : public AllowForbidAllCollisions
{
public:
  AllowAllCollisions(const std::string& name, const BT::NodeConfig& config)
    : AllowForbidAllCollisions(name, config, true)
  {}
};

class ForbidAllCollisions : public AllowForbidAllCollisions
{
public:
  ForbidAllCollisions(const std::string& name, const BT::NodeConfig& config)
    : AllowForbidAllCollisions(name, config, false)
  {}
};

class AllowCollisionsJointModelGroup : public AllowForbidCollisionsJointModelGroup
{
public:
  AllowCollisionsJointModelGroup(const std::string& name, const BT::NodeConfig& config)
    : AllowForbidCollisionsJointModelGroup(name, config, true)
  {}
};

class ForbidCollisionsJointModelGroup : public AllowForbidCollisionsJointModelGroup
{
public:
  ForbidCollisionsJointModelGroup(const std::string& name, const BT::NodeConfig& config)
    : AllowForbidCollisionsJointModelGroup(name, config, false)
  {}
};

}  // namespace MTC
}  // namespace BT
