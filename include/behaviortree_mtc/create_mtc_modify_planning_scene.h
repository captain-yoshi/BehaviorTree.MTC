#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace bt_mtc
{
class AttachDetachObjects : public BT::SyncActionNode
{
public:
  AttachDetachObjects(const std::string& name, const BT::NodeConfig& config, bool attach);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  const bool _attach;  // true = attach, false = detach
};

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

}  // namespace bt_mtc
