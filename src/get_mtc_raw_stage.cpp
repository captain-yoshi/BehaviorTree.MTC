#include <behaviortree_mtc/get_mtc_raw_stage.h>

#include <moveit/task_constructor/stage.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

namespace
{
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
    if(auto* stage_ptr = any_stage_ptr->castPtr<MTC::StagePtr>())
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
    BT::InputPort<MTC::StagePtr>(kPortStage, "MTC stage"),
    BT::OutputPort<MTC::Stage*>(kPortRawStage, "MTC stage raw pointer"),
  };
}
