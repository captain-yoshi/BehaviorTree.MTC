#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <moveit/task_constructor/stage.h>

namespace bt_mtc
{
template <typename T>
using SharedProperty = std::shared_ptr<T>;

template <typename T = BT::Any>
class SetMTCProperties : public BT::SyncActionNode
{
public:
  SetMTCProperties(const std::string& name, const BT::NodeConfig& config)
    : SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  {
    std::string property_name;
    SharedProperty<T> property;

    if(!getInput("property_name", property_name) ||
       !getInput("property", property))
      return BT::NodeStatus::FAILURE;

    if(auto any_locked = getLockedPortContent("stage");
       !any_locked.get()->empty())
    {
      if(auto stage = any_locked->template cast<moveit::task_constructor::StagePtr>())
      {
        // Store the value inside the shared pointer
        stage->properties().set(property_name, *property);

        return BT::NodeStatus::SUCCESS;
      }
    }

    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<moveit::task_constructor::StagePtr>("stage"),
      BT::InputPort<std::string>("property_name"),
      BT::InputPort<SharedProperty<T>>("property"),
    };
  }
};

}  // namespace bt_mtc

namespace BT
{
template <>
inline bt_mtc::SharedProperty<int> convertFromString<bt_mtc::SharedProperty<int>>(BT::StringView str)
{
  bt_mtc::SharedProperty<int> output = std::make_shared<int>(convertFromString<int>(str));
  return output;
}

template <>
inline bt_mtc::SharedProperty<bool> convertFromString<bt_mtc::SharedProperty<bool>>(BT::StringView str)
{
  bt_mtc::SharedProperty<bool> output = std::make_shared<bool>(convertFromString<bool>(str));
  return output;
}

template <>
inline bt_mtc::SharedProperty<double> convertFromString<bt_mtc::SharedProperty<double>>(BT::StringView str)
{
  bt_mtc::SharedProperty<double> output = std::make_shared<double>(convertFromString<double>(str));
  return output;
}

template <>
inline bt_mtc::SharedProperty<std::string>
convertFromString<bt_mtc::SharedProperty<std::string>>(BT::StringView str)
{
  bt_mtc::SharedProperty<std::string> output = std::make_shared<std::string>(convertFromString<std::string>(str));
  return output;
}

}  // namespace BT
