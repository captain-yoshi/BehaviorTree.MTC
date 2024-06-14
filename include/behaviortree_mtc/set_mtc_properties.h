#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <moveit/task_constructor/stage.h>

namespace bt_mtc
{
template <typename S, typename T = BT::Any>
class SetMTCProperties : public BT::SyncActionNode
{
  static_assert(std::is_base_of<moveit::task_constructor::Stage, S>::value, "S must inherit from moveit::task_constructor::Stage");

public:
  SetMTCProperties(const std::string& name, const BT::NodeConfig& config)
    : SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  {
    std::string property_name;
    T property;

    if(!getInput("property_name", property_name) ||
       !getInput("property", property))
      return BT::NodeStatus::FAILURE;

    if(auto any_locked = getLockedPortContent("stage");
       !any_locked.get()->empty())
    {
      if(auto stage = any_locked->template cast<std::shared_ptr<S>>())
      {
        stage->properties().set(property_name, property);

        return BT::NodeStatus::SUCCESS;
      }
    }

    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<std::shared_ptr<S>>("stage"),
      BT::InputPort<std::string>("property_name"),
      BT::InputPort<T>("property"),
    };
  }
};

}  // namespace bt_mtc
