#include <behaviortree_mtc/move_mtc_stage_to_container.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/task.h>

namespace BT {
namespace MTC {

MoveMTCStageToContainer::MoveMTCStageToContainer(const std::string& name,
                                                 const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus MoveMTCStageToContainer::tick()
{
  // Transform stage from shared to unique
  moveit::task_constructor::Stage::pointer unique_stage{ nullptr };
  if(auto any_stage_ptr = getLockedPortContent("stage"))
  {
    if(auto* stage_ptr = any_stage_ptr->castPtr<moveit::task_constructor::StagePtr>())
    {
      auto& stage = *stage_ptr;

      unique_stage = sharedToUnique(stage);
      any_stage_ptr.assign(nullptr);  // set blackboard value to nullptr
    }
  }

  if(auto any_container_ptr = getLockedPortContent("container"); unique_stage)
  {
    if(auto* container_ptr = any_container_ptr->castPtr<moveit::task_constructor::ContainerBasePtr>())
    {
      auto& container = *container_ptr;

      container->add(std::move(unique_stage));

      return NodeStatus::SUCCESS;
    }
  }

  return NodeStatus::FAILURE;
}

BT::PortsList MoveMTCStageToContainer::providedPorts()
{
  return {
    BT::InputPort<moveit::task_constructor::StagePtr>("stage"),
    BT::BidirectionalPort<moveit::task_constructor::ContainerBasePtr>("container"),
  };
}

}  // namespace MTC
}  // namespace BT
