#include <behaviortree_mtc/move_mtc_stage_to_container.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/container.h>

using namespace BT;
namespace MTC = moveit::task_constructor;

namespace
{
constexpr auto kPortContainer = "container";
constexpr auto kPortStage = "stage";

}  // namespace

MoveMTCStageToContainer::MoveMTCStageToContainer(const std::string& name,
                                                 const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus MoveMTCStageToContainer::tick()
{
  // Transform stage from shared to unique
  MTC::Stage::pointer unique_stage{ nullptr };
  if(auto any_stage_ptr = getLockedPortContent(kPortStage))
  {
    if(auto* stage_ptr = any_stage_ptr->castPtr<MTC::StagePtr>())
    {
      auto& stage = *stage_ptr;

      unique_stage = sharedToUnique(stage);
      any_stage_ptr.assign(nullptr);  // set blackboard value to nullptr
    }
  }

  if(auto any_container_ptr = getLockedPortContent(kPortContainer); unique_stage)
  {
    if(auto* container_ptr = any_container_ptr->castPtr<MTC::ContainerBase*>())
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
    BT::InputPort<MTC::StagePtr>(kPortStage),
    BT::BidirectionalPort<MTC::ContainerBase*>(kPortContainer),
  };
}
