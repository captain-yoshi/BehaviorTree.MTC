#include <behaviortree_mtc/create_mtc_generate_grasp_pose.h>
#include <behaviortree_mtc/custom_types.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/stages/generate_grasp_pose.h>

namespace BT {
namespace MTC {

CreateMTCGenerateGraspPose::CreateMTCGenerateGraspPose(const std::string& name,
                                                       const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreateMTCGenerateGraspPose::tick()
{
  // Retrieve inputs
  std::string name, eef, object, pregrasp_pose;
  double angle_delta;
  Vector3D rotation_axis_input;
  Eigen::Vector3d rotation_axis;
  moveit::task_constructor::Stage* monitored_stage;
  if(!getInput("stage_name", name) ||
     !getInput("eef", eef) ||
     !getInput("angle_delta", angle_delta) ||
     !getInput("object", object) ||
     !getInput("pregrasp_pose", pregrasp_pose) ||
     !getInput("monitored_stage", monitored_stage) ||
     !getInput("rotation_axis", rotation_axis_input))
    return NodeStatus::FAILURE;

  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::GenerateGraspPose> stage{ nullptr };
  stage = {
    new moveit::task_constructor::stages::GenerateGraspPose(name),
    dirty::fake_deleter{}
  };
  rotation_axis[0] = rotation_axis_input.x;
  rotation_axis[1] = rotation_axis_input.y;
  rotation_axis[2] = rotation_axis_input.z;
  stage->setEndEffector(eef);
  stage->setObject(object);
  stage->setAngleDelta(angle_delta);
  stage->setRotationAxis(rotation_axis);
  stage->setPreGraspPose(pregrasp_pose);
  stage->setMonitoredStage(monitored_stage);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = stage;

  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCGenerateGraspPose::providedPorts()
{
  return {
    BT::InputPort<std::string>("eef", "name of end-effector"),
    BT::InputPort<double>("angle_delta", 0.1, "angular steps (rad)"),
    BT::InputPort<std::string>("object", "object on which we generate the grasp poses"),
    BT::InputPort<Vector3D>("rotation_axis", "0,0,1", "rotate object pose about given axis"),
    BT::InputPort<std::string>("pregrasp_pose", "pregrasp posture"),
    BT::InputPort<std::string>("stage_name"),
    BT::InputPort<moveit::task_constructor::Stage*>("monitored_stage"),
    BT::OutputPort<moveit::task_constructor::StagePtr>("stage", "{generate_grasp_pose}", "GenerateGraspPose Stage"),
  };
}

}  // namespace MTC
}  // namespace BT
