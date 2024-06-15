#include <behaviortree_mtc/create_mtc_serial_container.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>

namespace BT {
namespace MTC {

namespace {
constexpr auto kPortSerialContainer = "serial_container";
constexpr auto kPortContainerName = "container_name";
}  // namespace

CreateMTCSerialContainer::CreateMTCSerialContainer(const std::string& name,
                                                   const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreateMTCSerialContainer::tick()
{
  // Inputs
  std::string container_name;
  if(!getInput(kPortContainerName, container_name))
    return NodeStatus::FAILURE;
  // Build
  std::shared_ptr<moveit::task_constructor::SerialContainer> serialContainer{
    new moveit::task_constructor::SerialContainer(container_name),
    dirty::fake_deleter{}
  };

  // Upcast to base class
  moveit::task_constructor::ContainerBasePtr serialContainerBase = serialContainer;
  setOutput(kPortSerialContainer, serialContainerBase);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCSerialContainer::providedPorts()
{
  return {
    BT::OutputPort<moveit::task_constructor::ContainerBasePtr>(kPortSerialContainer, "serial container"),
    BT::InputPort<std::string>(kPortContainerName, "serial container"),
  };
}

}  // namespace MTC
}  // namespace BT
