#include <behaviortree_mtc/initialize_mtc_task.h>

#include <moveit/task_constructor/task.h>

namespace BT {
namespace MTC {

InitializeMTCTask::InitializeMTCTask(const std::string& name,
                                     const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus InitializeMTCTask::tick()
{
  std::string task_name;

  if(!getInput("task_name", task_name))
    return NodeStatus::FAILURE;

  auto task = std::make_shared<moveit::task_constructor::Task>();
  task->setName(task_name);
  task->loadRobotModel();

  setOutput("task", task);

  return NodeStatus::SUCCESS;
}

BT::PortsList InitializeMTCTask::providedPorts()
{
  return {
    BT::OutputPort<moveit::task_constructor::TaskPtr>("task", "{mtc_task}",
                                                      "MoveIt Task Constructor task."),
    BT::InputPort<std::string>("task_name", "task pipeline", "MoveIt Task Constructor task name.")
  };
}

}  // namespace MTC
}  // namespace BT
