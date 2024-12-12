#include <behaviortree_mtc/initialize_mtc_task.h>

#include <moveit/task_constructor/task.h>

namespace BT {
namespace MTC {

InitializeMTCTask::InitializeMTCTask(const std::string& name,
                                     const BT::NodeConfig& config, 
                                     const BT::RosNodeParams& params)
  : SyncActionNode(name, config), node_(params.nh.lock())
{}

BT::NodeStatus InitializeMTCTask::tick()
{
  auto task = std::make_shared<moveit::task_constructor::Task>();
  task->loadRobotModel(node_);

  setOutput("task", task);

  return NodeStatus::SUCCESS;
}

BT::PortsList InitializeMTCTask::providedPorts()
{
  return {
    BT::OutputPort<moveit::task_constructor::TaskPtr>("task", "{mtc_task}",
                                                      "MoveIt Task Constructor task."),
  };
}

}  // namespace MTC
}  // namespace BT
