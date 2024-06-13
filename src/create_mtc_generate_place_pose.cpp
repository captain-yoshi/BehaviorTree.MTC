#include <ros/ros.h>
#include <behaviortree_mtc/create_mtc_generate_place_pose.h>
#include <behaviortree_mtc/custom_types.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/stages/generate_place_pose.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

namespace
{
constexpr auto kPortStage = "stage";
constexpr auto kPortStageName = "stage_name";
constexpr auto kPortObject = "object";
constexpr auto kPortIKFrame = "ik_frame";
constexpr auto kPortPlacePose = "place_pose";
constexpr auto kPortMonitoredStage = "monitored_stage";
}  // namespace

CreateMTCGeneratePlacePose::CreateMTCGeneratePlacePose(const std::string& name,
                                                       const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreateMTCGeneratePlacePose::tick()
{
  // Retrieve inputs
  std::string name, object;
  MTC::Stage* monitored_stage;
  std::shared_ptr<geometry_msgs::PoseStamped> pose{ nullptr };
  std::shared_ptr<geometry_msgs::PoseStamped> ik_frame{ nullptr };
  if(!getInput(kPortStageName, name) ||
     !getInput(kPortPlacePose, pose) ||
     !getInput(kPortObject, object) ||
     !getInput(kPortMonitoredStage, monitored_stage) ||
     !getInput(kPortIKFrame, ik_frame))
    return NodeStatus::FAILURE;

  // Build stage
  std::shared_ptr<MTC::stages::GeneratePlacePose> stage{ nullptr };
  stage = {
    new MTC::stages::GeneratePlacePose(name),
    dirty::fake_deleter{}
  };
  stage->setObject(object);
  stage->setProperty(kPortIKFrame, *ik_frame);
  stage->setMonitoredStage(monitored_stage);
  stage->setPose(*pose);

  // Upcast to base class
  MTC::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCGeneratePlacePose::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortObject, "object on which we generate the place poses"),
    BT::InputPort<std::string>(kPortStageName),
    BT::InputPort<MTC::Stage*>(kPortMonitoredStage),
    BT::InputPort<std::shared_ptr<geometry_msgs::PoseStamped>>(kPortIKFrame),
    BT::InputPort<std::shared_ptr<geometry_msgs::PoseStamped>>(kPortPlacePose),
    BT::OutputPort<MTC::StagePtr>(kPortStage, "{generate_grasp_pose}", "GenerateGraspPose Stage"),
  };
}
