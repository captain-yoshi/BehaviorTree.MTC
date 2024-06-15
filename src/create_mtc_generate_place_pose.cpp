#include <ros/ros.h>
#include <behaviortree_mtc/create_mtc_generate_place_pose.h>
#include <behaviortree_mtc/custom_types.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/stages/generate_place_pose.h>

namespace BT {
namespace MTC {

namespace {
constexpr auto kPortStage = "stage";
constexpr auto kPortStageName = "stage_name";
constexpr auto kPortObject = "object";
constexpr auto kPortPose = "pose";
constexpr auto kPortMonitoredStage = "monitored_stage";
constexpr auto kPortAllowZFlip = "allow_z_flip";
}  // namespace

CreateMTCGeneratePlacePose::CreateMTCGeneratePlacePose(const std::string& name,
                                                       const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreateMTCGeneratePlacePose::tick()
{
  // Retrieve inputs
  std::string name, object;
  moveit::task_constructor::Stage* monitored_stage;
  bool allow_z_flip;
  std::shared_ptr<geometry_msgs::PoseStamped> pose{ nullptr };
  if(!getInput(kPortStageName, name) ||
     !getInput(kPortPose, pose) ||
     !getInput(kPortObject, object) ||
     !getInput(kPortAllowZFlip, allow_z_flip) ||
     !getInput(kPortMonitoredStage, monitored_stage))
    return NodeStatus::FAILURE;

  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::GeneratePlacePose> stage{ nullptr };
  stage = {
    new moveit::task_constructor::stages::GeneratePlacePose(name),
    dirty::fake_deleter{}
  };
  stage->setObject(object);
  stage->setMonitoredStage(monitored_stage);
  stage->setProperty(kPortAllowZFlip, allow_z_flip);
  stage->setPose(*pose);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCGeneratePlacePose::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortObject, "object on which we generate the place poses"),
    BT::InputPort<std::string>(kPortStageName),
    BT::InputPort<moveit::task_constructor::Stage*>(kPortMonitoredStage),
    BT::InputPort<std::shared_ptr<geometry_msgs::PoseStamped>>(kPortPose, "target pose to pass on in spawned states"),
    BT::InputPort<bool>(kPortAllowZFlip, false, "allow placing objects upside down"),
    BT::OutputPort<moveit::task_constructor::StagePtr>(kPortStage, "{generate_grasp_pose}", "GenerateGraspPose Stage"),
  };
}

}  // namespace MTC
}  // namespace BT