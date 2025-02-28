#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace BT {
namespace MTC {

class GeneratePlacePose : public BT::SyncActionNode
{
public:
  GeneratePlacePose(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

}  // namespace MTC
}  // namespace BT
