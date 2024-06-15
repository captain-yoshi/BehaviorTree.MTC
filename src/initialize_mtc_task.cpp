#include <behaviortree_mtc/initialize_mtc_task.h>

#include <moveit/task_constructor/task.h>

namespace BT {
namespace MTC {

namespace {
constexpr auto kPortTask = "task";
constexpr auto kPortContainer = "container";

}  // namespace

InitializeMTCTask::InitializeMTCTask(const std::string& name,
                                     const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus InitializeMTCTask::tick()
{
  auto task = std::make_shared<moveit::task_constructor::Task>();
  task->loadRobotModel();

  //Convert raw pointer to share pointer
  auto container = std::shared_ptr<moveit::task_constructor::ContainerBase>(task->stages());

  setOutput(kPortTask, task);
  setOutput(kPortContainer, container);

  return NodeStatus::SUCCESS;
}

BT::PortsList InitializeMTCTask::providedPorts()
{
  return {
    BT::OutputPort<moveit::task_constructor::TaskPtr>(kPortTask, "{mtc_task}",
                                                      "MoveIt Task Constructor task."),
    BT::OutputPort<moveit::task_constructor::ContainerBasePtr>(kPortContainer),

  };
}

}  // namespace MTC
}  // namespace BT
