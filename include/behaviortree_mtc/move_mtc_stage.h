#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/task.h>

namespace BT {
namespace MTC {

/// Move MTC child stage/container to parent container/task
template <typename C, typename P>
class MoveMTCStage : public BT::SyncActionNode
{
  static_assert(std::is_base_of<moveit::task_constructor::Stage, C>::value, "C must inherit from moveit::task_constructor::Stage");
  static_assert(std::is_base_of<moveit::task_constructor::Stage, P>::value, "P must inherit from moveit::task_constructor::Stage");

public:
  MoveMTCStage(const std::string& name, const BT::NodeConfig& config)
    : SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  {
    // Transform stage from shared to unique
    moveit::task_constructor::Stage::pointer unique_child{ nullptr };
    if(auto any_child_ptr = getLockedPortContent("child"))
    {
      if(auto* stage_ptr = any_child_ptr->template castPtr<std::shared_ptr<C>>())
      {
        auto& stage = *stage_ptr;

        unique_child = sharedToUnique(stage);
        any_child_ptr.assign(nullptr);  // set blackboard value to nullptr
      }
    }

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

}  // namespace MTC
}  // namespace BT
