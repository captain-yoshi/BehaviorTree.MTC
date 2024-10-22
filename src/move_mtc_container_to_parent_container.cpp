#include <behaviortree_mtc/move_mtc_container_to_parent_container.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/task.h>

namespace BT {
namespace MTC {

MoveMTCContainerToParentContainer::MoveMTCContainerToParentContainer(const std::string& name,
                                                                     const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus MoveMTCContainerToParentContainer::tick()
{
  // Transform container from shared to unique
  moveit::task_constructor::ContainerBase::pointer unique_child_container{ nullptr };
  if(auto any_container_ptr = getLockedPortContent("child_container"))
  {
    if(auto* container_ptr = any_container_ptr->castPtr<moveit::task_constructor::ContainerBasePtr>())
    {
      auto& child_container = *container_ptr;

      unique_child_container = sharedToUnique(child_container);
      any_container_ptr.assign(nullptr);  // set blackboard value to nullptr
    }
  }

  if(auto any_container_ptr = getLockedPortContent("parent_container"); unique_child_container)
  {
    if(auto* container_ptr = any_container_ptr->castPtr<moveit::task_constructor::ContainerBasePtr>())
    {
      auto& parent_container = *container_ptr;

      parent_container->add(std::move(unique_child_container));

      return NodeStatus::SUCCESS;
    }
  }

  return NodeStatus::SUCCESS;
}

BT::PortsList MoveMTCContainerToParentContainer::providedPorts()
{
  return {
    BT::InputPort<moveit::task_constructor::ContainerBasePtr>("child_container"),
    BT::BidirectionalPort<moveit::task_constructor::ContainerBasePtr>("parent_container"),
  };
}

}  // namespace MTC
}  // namespace BT
