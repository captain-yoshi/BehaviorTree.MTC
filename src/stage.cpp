#include <behaviortree_mtc/stage.h>

#include <moveit/task_constructor/stage.h>

namespace BT {
namespace MTC {

namespace {
constexpr auto kPortStage = "stage";
constexpr auto kPortRawStage = "raw_stage";

}  // namespace

StageGetRawPtr::StageGetRawPtr(const std::string& name,
                               const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus StageGetRawPtr::tick()
{
  // Convert stage shared pointer to raw pointer
  if(auto any_stage_ptr = getLockedPortContent("stage"))
  {
    if(auto* stage_ptr = any_stage_ptr->castPtr<moveit::task_constructor::StagePtr>())
    {
      auto& stage = *stage_ptr;

      setOutput("raw_stage", stage.get());
      return NodeStatus::SUCCESS;
    }
  }

  return NodeStatus::FAILURE;
}

BT::PortsList StageGetRawPtr::providedPorts()
{
  return {
    BT::InputPort<moveit::task_constructor::StagePtr>("stage", "MTC stage"),
    BT::OutputPort<moveit::task_constructor::Stage*>("raw_stage", "MTC stage raw pointer"),
  };
}

}  // namespace MTC
}  // namespace BT
