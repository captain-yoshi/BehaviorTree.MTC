#include <behaviortree_mtc/initialize_mtc_task.h>

#include <moveit/task_constructor/task.h>

using namespace BT;
namespace MTC = moveit::task_constructor;

namespace
{
constexpr auto kPortTask = "task";

}  // namespace

InitializeMTCTask::InitializeMTCTask(const std::string& name,
                                     const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus InitializeMTCTask::tick()
{
  if(auto any_ptr = getLockedPortContent(kPortTask))
  {
    std::shared_ptr<MTC::Task> task = std::make_shared<MTC::Task>();
    task->loadRobotModel();

    any_ptr.assign(task);

    return NodeStatus::SUCCESS;
  }
  else
  {
    return NodeStatus::FAILURE;
  }
}

BT::PortsList InitializeMTCTask::providedPorts()
{
  return {
    BT::OutputPort<std::shared_ptr<MTC::Task>>(kPortTask, "{mtc_task}",
                                               "MoveIt Task Constructor task."),
  };
}
