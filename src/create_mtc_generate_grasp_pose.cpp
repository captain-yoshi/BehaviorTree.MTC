#include <behaviortree_mtc/create_mtc_generate_grasp_pose.h>
#include <behaviortree_mtc/custom_types.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/stages/generate_grasp_pose.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

namespace
{
constexpr auto kPortStage = "stage";
constexpr auto kPortStageName = "stage_name";
constexpr auto kPortEef = "eef";
constexpr auto kPortObject = "object";
constexpr auto kPortAngleDelta = "angle_delta";
constexpr auto kPortRotationAxis = "rotation_axis";
constexpr auto kPortPreGraspPose = "pregrasp_pose";
constexpr auto kPortMonitoredStage = "monitored_stage";
}  // namespace

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
  MTC::Stage* monitored_stage;
  if(!getInput(kPortStageName, name) ||
     !getInput(kPortEef, eef) ||
     !getInput(kPortAngleDelta, angle_delta) ||
     !getInput(kPortObject, object) ||
     !getInput(kPortPreGraspPose, pregrasp_pose) ||
     !getInput(kPortMonitoredStage, monitored_stage) ||
     !getInput(kPortRotationAxis, rotation_axis_input))
    return NodeStatus::FAILURE;
    
  // Build stage
  std::shared_ptr<MTC::stages::GenerateGraspPose> stage{ nullptr };
  stage = {
    new MTC::stages::GenerateGraspPose(name),
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
  MTC::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCGenerateGraspPose::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortEef, "name of end-effector"),
    BT::InputPort<double>(kPortAngleDelta, 0.1, "angular steps (rad)"),
    BT::InputPort<std::string>(kPortObject, "object on which we generate the grasp poses"),
    BT::InputPort<Vector3D>(kPortRotationAxis, "0,0,1", "rotate object pose about given axis"),
    BT::InputPort<std::string>(kPortPreGraspPose, "pregrasp posture"),
    BT::InputPort<std::string>(kPortStageName),
    BT::InputPort<MTC::Stage*>(kPortMonitoredStage),
    BT::OutputPort<MTC::StagePtr>(kPortStage, "{generate_grasp_pose}", "GenerateGraspPose Stage"),
  };
}
