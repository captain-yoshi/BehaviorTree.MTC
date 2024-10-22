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

  //Convert raw pointer to share pointer
  auto container = std::shared_ptr<moveit::task_constructor::ContainerBase>(task->stages());

  setOutput("task", task);
  setOutput("container", container);

  return NodeStatus::SUCCESS;
}

BT::PortsList InitializeMTCTask::providedPorts()
{
  return {
    BT::OutputPort<moveit::task_constructor::TaskPtr>("task", "{mtc_task}",
                                                      "MoveIt Task Constructor task."),
    BT::OutputPort<moveit::task_constructor::ContainerBasePtr>("container"),

  };
}

}  // namespace MTC
}  // namespace BT
