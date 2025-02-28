#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_mtc/bt/blackboard_helper.h>

#include <moveit/task_constructor/task.h>

namespace BT {
namespace MTC {

class StageGetRawPtr : public BT::SyncActionNode
{
public:
  StageGetRawPtr(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

/// Move MTC child stage/container to parent container/task
template <typename C, typename P>
class StageMove : public BT::SyncActionNode
{
  static_assert(std::is_base_of<moveit::task_constructor::Stage, C>::value, "C must inherit from moveit::task_constructor::Stage");
  static_assert(std::is_base_of<moveit::task_constructor::Stage, P>::value, "P must inherit from moveit::task_constructor::Stage");

public:
  StageMove(const std::string& name, const BT::NodeConfig& config)
    : SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  {
    // Transform stage: shared -> unique
    auto unique_child = convertSharedToUniqueLocked<C>(*this, "child");

    if(auto any_parent_ptr = getLockedPortContent("parent"); unique_child)
    {
      if(auto* parent_ptr = any_parent_ptr->template castPtr<std::shared_ptr<P>>())
      {
        auto& parent = *parent_ptr;

        parent->add(std::move(unique_child));

        return NodeStatus::SUCCESS;
      }
    }

    return NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<C>>("child"),
      BT::BidirectionalPort<std::shared_ptr<P>>("parent"),
    };
  }
};

template <typename S, typename T = BT::Any>
class PropertiesSet : public BT::SyncActionNode
{
  static_assert(std::is_base_of<moveit::task_constructor::Stage, S>::value, "S must inherit from moveit::task_constructor::Stage");

public:
  PropertiesSet(const std::string& name, const BT::NodeConfig& config)
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

template <typename FROM_S, typename TO_S>
class PropertiesExposeTo : public BT::SyncActionNode
{
  static_assert(std::is_base_of<moveit::task_constructor::Stage, FROM_S>::value, "FROM_S must inherit from moveit::task_constructor::Stage");
  static_assert(std::is_base_of<moveit::task_constructor::Stage, TO_S>::value, "TO_S must inherit from moveit::task_constructor::Stage");

public:
  PropertiesExposeTo(const std::string& name, const BT::NodeConfig& config)
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

template <typename S>
class PropertiesConfigureInitFrom : public BT::SyncActionNode
{
  static_assert(std::is_base_of<moveit::task_constructor::Stage, S>::value, "S must inherit from moveit::task_constructor::Stage");

public:
  PropertiesConfigureInitFrom(const std::string& name, const BT::NodeConfig& config)
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
