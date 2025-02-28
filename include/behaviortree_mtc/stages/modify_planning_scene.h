#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace BT {
namespace MTC {

/// Base class
class AttachDetachObjectsBase : public BT::SyncActionNode
{
public:
  AttachDetachObjectsBase(const std::string& name, const BT::NodeConfig& config, bool attach);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  const bool _attach;  // true = attach, false = detach
};

class AllowForbidCollisionsBase : public BT::SyncActionNode
{
public:
  AllowForbidCollisionsBase(const std::string& name, const BT::NodeConfig& config, bool allow);

  BT::NodeStatus tick(const std::vector<std::string>& first, const std::vector<std::string>& second);
  static BT::PortsList providedPorts();

private:
  const bool _allow;  // true = allow collisions, false = forbid collisions
};

class AllowForbidCollisionPairsBase : public AllowForbidCollisionsBase
{
public:
  AllowForbidCollisionPairsBase(const std::string& name, const BT::NodeConfig& config, bool allow);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class AllowForbidAllCollisionsBase : public AllowForbidCollisionsBase
{
public:
  AllowForbidAllCollisionsBase(const std::string& name, const BT::NodeConfig& config, bool allow);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class AllowForbidCollisionsJointModelGroupBase : public AllowForbidCollisionsBase
{
public:
  AllowForbidCollisionsJointModelGroupBase(const std::string& name, const BT::NodeConfig& config, bool allow);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

/// Actual BT nodes
class ModifyPlanningSceneAttachObjects : public AttachDetachObjectsBase
{
public:
  ModifyPlanningSceneAttachObjects(const std::string& name, const BT::NodeConfig& config)
    : AttachDetachObjectsBase(name, config, true)
  {}
};

class ModifyPlanningSceneDetachObjects : public AttachDetachObjectsBase
{
public:
  ModifyPlanningSceneDetachObjects(const std::string& name, const BT::NodeConfig& config)
    : AttachDetachObjectsBase(name, config, false)
  {}
};

class ModifyPlanningSceneAllowCollisionPairs : public AllowForbidCollisionPairsBase
{
public:
  ModifyPlanningSceneAllowCollisionPairs(const std::string& name, const BT::NodeConfig& config)
    : AllowForbidCollisionPairsBase(name, config, true)
  {}
};

class ModifyPlanningSceneForbidCollisionPairs : public AllowForbidCollisionPairsBase
{
public:
  ModifyPlanningSceneForbidCollisionPairs(const std::string& name, const BT::NodeConfig& config)
    : AllowForbidCollisionPairsBase(name, config, false)
  {}
};

class ModifyPlanningSceneAllowAllCollisions : public AllowForbidAllCollisionsBase
{
public:
  ModifyPlanningSceneAllowAllCollisions(const std::string& name, const BT::NodeConfig& config)
    : AllowForbidAllCollisionsBase(name, config, true)
  {}
};

class ModifyPlanningSceneForbidAllCollisions : public AllowForbidAllCollisionsBase
{
public:
  ModifyPlanningSceneForbidAllCollisions(const std::string& name, const BT::NodeConfig& config)
    : AllowForbidAllCollisionsBase(name, config, false)
  {}
};

class ModifyPlanningSceneAllowCollisionsJointModelGroup : public AllowForbidCollisionsJointModelGroupBase
{
public:
  ModifyPlanningSceneAllowCollisionsJointModelGroup(const std::string& name, const BT::NodeConfig& config)
    : AllowForbidCollisionsJointModelGroupBase(name, config, true)
  {}
};

class ModifyPlanningSceneForbidCollisionsJointModelGroup : public AllowForbidCollisionsJointModelGroupBase
{
public:
  ModifyPlanningSceneForbidCollisionsJointModelGroup(const std::string& name, const BT::NodeConfig& config)
    : AllowForbidCollisionsJointModelGroupBase(name, config, false)
  {}
};

}  // namespace MTC
}  // namespace BT
