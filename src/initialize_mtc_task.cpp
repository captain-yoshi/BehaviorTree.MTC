#include <behaviortree_mtc/initialize_mtc_task.h>

#include <moveit/task_constructor/task.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

namespace
{
constexpr auto kPortTask = "task";
constexpr auto kPortContainer = "container";

}  // namespace

InitializeMTCTask::InitializeMTCTask(const std::string& name,
                                     const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus InitializeMTCTask::tick()
{
  std::shared_ptr<MTC::Task> task = std::make_shared<MTC::Task>();
  task->loadRobotModel();

  setOutput(kPortTask, task);
  setOutput(kPortContainer, task->stages());

  return NodeStatus::SUCCESS;
}

BT::PortsList InitializeMTCTask::providedPorts()
{
  return {
    BT::OutputPort<MTC::TaskPtr>(kPortTask, "{mtc_task}",
                                 "MoveIt Task Constructor task."),
    BT::OutputPort<MTC::ContainerBase*>(kPortContainer),

  };
}
