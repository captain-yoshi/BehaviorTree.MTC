#include <behaviortree_mtc/stages/current_state.h>
#include <behaviortree_mtc/bt/shared_to_unique.h>

#include <moveit/task_constructor/stages/current_state.h>

namespace BT {
namespace MTC {

CurrentState::CurrentState(const std::string& name,
                           const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CurrentState::tick()
{
  std::shared_ptr<moveit::task_constructor::stages::CurrentState> stage{
    new moveit::task_constructor::stages::CurrentState("current state"),
    dirty::fake_deleter{}
  };

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CurrentState::providedPorts()
{
  return {
    BT::OutputPort<moveit::task_constructor::StagePtr>("stage", "{mtc_stage}",
                                                       "MoveIt Task Constructor stage."),
  };
}

}  // namespace MTC
}  // namespace BT
