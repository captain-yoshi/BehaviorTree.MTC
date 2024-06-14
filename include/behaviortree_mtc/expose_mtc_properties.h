#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <moveit/task_constructor/task.h>

namespace bt_mtc
{
template <typename FROM_S, typename TO_S>
class ExposeMTCProperties : public BT::SyncActionNode
{
  static_assert(std::is_base_of<moveit::task_constructor::Stage, FROM_S>::value, "FROM_S must inherit from moveit::task_constructor::Stage");
  static_assert(std::is_base_of<moveit::task_constructor::Stage, TO_S>::value, "TO_S must inherit from moveit::task_constructor::Stage");

public:
  ExposeMTCProperties(const std::string& name, const BT::NodeConfig& config)
    : SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  {
    std::shared_ptr<FROM_S> from{ nullptr };
    std::shared_ptr<TO_S> to{ nullptr };
    std::shared_ptr<std::set<std::string>> property_names{ nullptr };

    if(!getInput("property_names", property_names) ||
       !getInput("from", from) ||
       !getInput("to", to))
      return BT::NodeStatus::FAILURE;

    from->properties().exposeTo(to->properties(), *property_names);

    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<std::set<std::string>>>("property_names"),

      BT::BidirectionalPort<std::shared_ptr<FROM_S>>("from"),
      BT::BidirectionalPort<std::shared_ptr<TO_S>>("to"),
    };
  }
};

}  // namespace bt_mtc
