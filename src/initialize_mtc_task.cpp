#include <behaviortree_mtc/initialize_mtc_task.h>

#include <moveit/task_constructor/task.h>

#include <rclcpp/node.hpp>

namespace BT {
namespace MTC {

InitializeMTCTask::InitializeMTCTask(const std::string& name,
                                     const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus InitializeMTCTask::tick()
{
  auto task = std::make_shared<moveit::task_constructor::Task>();

  auto node = std::make_shared<rclcpp::Node>("initialize_mtc_task");
  task->loadRobotModel(node);

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
