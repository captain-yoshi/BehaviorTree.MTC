#include <behaviortree_mtc/create_mtc_current_state.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/stages/current_state.h>

namespace BT {
namespace MTC {

namespace {
constexpr auto kPortStage = "stage";

}  // namespace

CreateMTCCurrentState::CreateMTCCurrentState(const std::string& name,
                                             const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreateMTCCurrentState::tick()
{
  std::shared_ptr<moveit::task_constructor::stages::CurrentState> stage{
    new moveit::task_constructor::stages::CurrentState("current state"),
    dirty::fake_deleter{}
  };

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCCurrentState::providedPorts()
{
  return {
    BT::OutputPort<moveit::task_constructor::StagePtr>(kPortStage, "{mtc_stage}",
                                                       "MoveIt Task Constructor stage."),
  };
}

}  // namespace MTC
}  // namespace BT
