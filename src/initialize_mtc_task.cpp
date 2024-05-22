#include <behaviortree_mtc/initialize_mtc_task.h>
#include <moveit/task_constructor/task.h>

using namespace BT;
namespace MTC = moveit::task_constructor;
using namespace bt_mtc;

// namespace
// {
// constexpr auto kPortTask = "task";

// }  // namespace

InitializeMTCTask::InitializeMTCTask(const std::string& name,
                                     const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus InitializeMTCTask::tick()
{
  if(auto any_ptr = getLockedPortContent("task"))
  {
    printf("initializing");
    std::shared_ptr<MTC::Task> task = std::make_shared<MTC::Task>();
    task->loadRobotModel(); //manque des robot model pis tt.. on va attendre DAVID... je propose qu'on ajuste tout de suite notre sampler en changenat le nom et mettre les inputs avec les defaults possible.

    any_ptr.assign(task);
    printf("task_intialized");
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
    BT::OutputPort<std::shared_ptr<MTC::Task>>("task", "{mtc_task}",
                                               "MoveIt Task Constructor task."),
  };
}
