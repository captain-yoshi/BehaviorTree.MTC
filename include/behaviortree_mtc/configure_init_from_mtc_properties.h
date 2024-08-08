#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <moveit/task_constructor/task.h>

namespace BT {
namespace MTC {

template <typename S>
class ConfigureInitFromMTCProperties : public BT::SyncActionNode
{
  static_assert(std::is_base_of<moveit::task_constructor::Stage, S>::value, "S must inherit from moveit::task_constructor::Stage");

public:
  ConfigureInitFromMTCProperties(const std::string& name, const BT::NodeConfig& config)
    : SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  {
    std::shared_ptr<S> stage{ nullptr };
    std::shared_ptr<std::set<std::string>> property_names{ nullptr };
    moveit::task_constructor::Stage::PropertyInitializerSource source_flag;

    if(!getInput("stage", stage) ||
       !getInput("property_names", property_names) ||
       !getInput("source_flag", source_flag))
      return BT::NodeStatus::FAILURE;

    stage->properties().configureInitFrom(source_flag, *property_names);

    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<moveit::task_constructor::Stage::PropertyInitializerSource>("source_flag", "Name for property initialization sources. {DEFAULT, MANUAL, PARENT, INTERFACE}"),
      BT::InputPort<std::shared_ptr<std::set<std::string>>>("property_names"),

      BT::BidirectionalPort<std::shared_ptr<S>>("stage"),
    };
  }
};

}  // namespace MTC
}  // namespace BT
