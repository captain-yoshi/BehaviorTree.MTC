#include <behaviortree_mtc/get_mtc_raw_stage.h>

#include <moveit/task_constructor/stage.h>

namespace BT {
namespace MTC {

namespace {
constexpr auto kPortStage = "stage";
constexpr auto kPortRawStage = "raw_stage";

}  // namespace

GetMTCRawStage::GetMTCRawStage(const std::string& name,
                               const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus GetMTCRawStage::tick()
{
  // Convert stage shared pointer to raw pointer
  if(auto any_stage_ptr = getLockedPortContent(kPortStage))
  {
    if(auto* stage_ptr = any_stage_ptr->castPtr<moveit::task_constructor::StagePtr>())
    {
      auto& stage = *stage_ptr;

      setOutput(kPortRawStage, stage.get());
      return NodeStatus::SUCCESS;
    }
  }

  return NodeStatus::FAILURE;
}

BT::PortsList GetMTCRawStage::providedPorts()
{
  return {
    BT::InputPort<moveit::task_constructor::StagePtr>(kPortStage, "MTC stage"),
    BT::OutputPort<moveit::task_constructor::Stage*>(kPortRawStage, "MTC stage raw pointer"),
  };
}

}  // namespace MTC
}  // namespace BT
