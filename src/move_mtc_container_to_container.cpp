#include <behaviortree_mtc/move_mtc_container_to_container.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/task.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

namespace
{
constexpr auto kPortParentContainer = "parent_container";
constexpr auto kPortChildContainer = "child_container";
}  // namespace

MoveMTCContainerToContainer::MoveMTCContainerToContainer(const std::string& name,
                                                         const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus MoveMTCContainerToContainer::tick()
{
  // Transform container from shared to unique
  MTC::ContainerBase::pointer unique_container_child{ nullptr };
  if(auto any_container_ptr = getLockedPortContent(kPortChildContainer))
  {
    if(auto* container_ptr = any_container_ptr->castPtr<MTC::ContainerBasePtr>())
    {
      auto& container = *container_ptr;

      unique_container_child = sharedToUnique(container);
      any_container_ptr.assign(nullptr);  // set blackboard value to nullptr
    }
  }

  if(auto any_container_ptr = getLockedPortContent(kPortParentContainer); unique_container_child)
  {
    if(auto* container_ptr = any_container_ptr->castPtr<MTC::ContainerBasePtr>())
    {
      auto& container = *container_ptr;

      container->add(std::move(unique_container_child));

      return NodeStatus::SUCCESS;
    }
  }

  return NodeStatus::FAILURE;
  return NodeStatus::SUCCESS;
}

BT::PortsList MoveMTCContainerToContainer::providedPorts()
{
  return {
    BT::InputPort<MTC::ContainerBasePtr>(kPortChildContainer),
    BT::BidirectionalPort<MTC::ContainerBasePtr>(kPortParentContainer),
  };
}
