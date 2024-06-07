#include <behaviortree_mtc/create_mtc_serial_container.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

namespace
{
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
  std::shared_ptr<MTC::SerialContainer> serialContainer{
    new MTC::SerialContainer(container_name),
    dirty::fake_deleter{}
  };

  // Upcast to base class
  MTC::ContainerBasePtr serialContainerBase = serialContainer;
  setOutput(kPortSerialContainer, serialContainerBase);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCSerialContainer::providedPorts()
{
  return {
    BT::OutputPort<MTC::ContainerBasePtr>(kPortSerialContainer, "serial container"),
    BT::InputPort<std::string>(kPortContainerName, "serial container"),
  };
}
