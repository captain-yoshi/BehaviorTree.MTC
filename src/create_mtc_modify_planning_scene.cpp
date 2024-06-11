#include <behaviortree_mtc/create_mtc_modify_planning_scene.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/stages/modify_planning_scene.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

AttachDetachObjects::AttachDetachObjects(const std::string& name, const BT::NodeConfig& config, bool attach)
  : SyncActionNode(name, config), _attach(attach)
{}

BT::NodeStatus AttachDetachObjects::tick()
{
  std::string link_name;
  std::string stage_name;
  std::vector<std::string> object_names;

  if(!getInput("link_name", link_name) ||
     !getInput("stage_name", stage_name) ||
     !getInput("object_names", object_names))
    return NodeStatus::FAILURE;

  // build stage
  std::shared_ptr<MTC::stages::ModifyPlanningScene> stage{ nullptr };
  stage = {
    new MTC::stages::ModifyPlanningScene(stage_name),
    dirty::fake_deleter{}
  };

  stage->attachObjects(object_names, link_name, _attach);

  // upcast to base class
  MTC::StagePtr base_stage = stage;

  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList AttachDetachObjects::providedPorts()
{
  return {
    BT::InputPort<std::string>("stage_name"),
    BT::InputPort<std::vector<std::string>>("object_names", "list of object names/id's."),
    BT::InputPort<std::vector<std::string>>("link_name", "attach or detach a list of objects to the given link"),
    BT::OutputPort<MTC::StagePtr>("stage", "ModifyPlanningScene stage"),
  };
}
