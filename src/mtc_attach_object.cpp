#include <behaviortree_mtc/mtc_attach_object.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/stages/modify_planning_scene.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

namespace
{
constexpr auto kPortFrame = "frame_id";
constexpr auto kPortObject = "object";
constexpr auto kPortStage = "stage";
constexpr auto kPortStageName = "stage_name";
}  // namespace

MTCAttachObject::MTCAttachObject(const std::string& name,
                                       const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus MTCAttachObject::tick()
{
  // Retrieve inputs
  std::string object, frame_id, name;

  if(!getInput(kPortStageName, name) ||
     !getInput(kPortObject, object) ||
     !getInput(kPortFrame, frame_id))
    return NodeStatus::FAILURE;

  // Build stage
  std::shared_ptr<MTC::stages::ModifyPlanningScene> stage{ nullptr };
  stage = {
    new MTC::stages::ModifyPlanningScene(name),
    dirty::fake_deleter{}
  };
  stage->attachObject(object, frame_id);

  // Upcast to base class
  MTC::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList MTCAttachObject::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortObject),
    BT::InputPort<std::string>(kPortFrame),
    BT::InputPort<std::string>(kPortStageName),
    BT::OutputPort<MTC::StagePtr>(kPortStage),
  };
}
