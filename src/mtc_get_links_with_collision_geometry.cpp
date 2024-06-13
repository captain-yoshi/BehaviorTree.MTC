#include <behaviortree_mtc/mtc_get_links_with_collision_geometry.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/task.h>
#include <moveit/robot_model/robot_model.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

namespace
{
constexpr auto kPortGroup = "group";
constexpr auto kPortTask = "task";
constexpr auto kPortLinks = "links";
}  // namespace

MTCGetLinksWithCollisionGeometry::MTCGetLinksWithCollisionGeometry(const std::string& name,
                                                                   const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus MTCGetLinksWithCollisionGeometry::tick()
{
  // Retrieve inputs
  std::string group;
  std::vector<std::string> links;

  if(!getInput(kPortGroup, group))
    return NodeStatus::FAILURE;
  if(auto any_task_ptr = getLockedPortContent(kPortTask))
  {
    if(auto* task_ptr = any_task_ptr->castPtr<MTC::TaskPtr>())
    {
      auto& task = *task_ptr;
      links = task->getRobotModel()->getJointModelGroup(group)->getLinkModelNamesWithCollisionGeometry();
      std::cout << "Link names with collision geometry:" << std::endl;
      for(const auto& name : links)
      {
        std::cout << name << std::endl;
      }

      setOutput(kPortLinks, links);

      return NodeStatus::SUCCESS;
    }
  }
  return NodeStatus::FAILURE;
}

BT::PortsList MTCGetLinksWithCollisionGeometry::providedPorts()
{
  return {
    BT::BidirectionalPort<MTC::TaskPtr>(kPortTask),
    BT::InputPort<std::string>(kPortGroup),
    BT::OutputPort<std::vector<std::string>>(kPortLinks),
  };
}
