#include <behaviortree_mtc/container.h>
#include <behaviortree_mtc/bt/shared_to_unique.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>

namespace BT {
namespace MTC {

SerialContainer::SerialContainer(const std::string& name,
                                                   const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus SerialContainer::tick()
{
  // Inputs
  std::string name;
  if(!getInput("container_name", name))
    return NodeStatus::FAILURE;
  // Build
  std::shared_ptr<moveit::task_constructor::SerialContainer> serial_container{
    new moveit::task_constructor::SerialContainer(name),
    dirty::fake_deleter{}
  };

  // Upcast to base class
  moveit::task_constructor::ContainerBasePtr container_base = serial_container;
  setOutput("container", container_base);

  return NodeStatus::SUCCESS;
}

BT::PortsList SerialContainer::providedPorts()
{
  return {
    BT::InputPort<std::string>("container_name", "serial container name"),
    BT::OutputPort<moveit::task_constructor::ContainerBasePtr>("container", "serial container"),
  };
}

}  // namespace MTC
}  // namespace BT
