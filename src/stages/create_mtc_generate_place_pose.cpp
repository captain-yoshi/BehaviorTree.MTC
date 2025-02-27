#include <ros/ros.h>
#include <behaviortree_mtc/stages/create_mtc_generate_place_pose.h>
#include <behaviortree_mtc/serialization/custom_types.h>
#include <behaviortree_mtc/bt/shared_to_unique.h>

#include <moveit/task_constructor/stages/generate_place_pose.h>

namespace BT {
namespace MTC {

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
  if(!getInput("stage_name", name) ||
     !getInput("pose", pose) ||
     !getInput("object", object) ||
     !getInput("allow_z_flip", allow_z_flip) ||
     !getInput("monitored_stage", monitored_stage))
    return NodeStatus::FAILURE;

  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::GeneratePlacePose> stage{ nullptr };
  stage = {
    new moveit::task_constructor::stages::GeneratePlacePose(name),
    dirty::fake_deleter{}
  };
  stage->setObject(object);
  stage->setMonitoredStage(monitored_stage);
  stage->setProperty("allow_z_flip", allow_z_flip);
  stage->setPose(*pose);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCGeneratePlacePose::providedPorts()
{
  return {
    BT::InputPort<std::string>("object", "object on which we generate the place poses"),
    BT::InputPort<std::string>("stage_name"),
    BT::InputPort<moveit::task_constructor::Stage*>("monitored_stage"),
    BT::InputPort<std::shared_ptr<geometry_msgs::PoseStamped>>("pose", "target pose to pass on in spawned states"),
    BT::InputPort<bool>("allow_z_flip", false, "allow placing objects upside down"),
    BT::OutputPort<moveit::task_constructor::StagePtr>("stage", "{generate_grasp_pose}", "GenerateGraspPose Stage"),
  };
}

}  // namespace MTC
}  // namespace BT
