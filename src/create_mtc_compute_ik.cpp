#include <behaviortree_mtc/create_mtc_compute_ik.h>
#include <behaviortree_mtc/custom_types.h>
#include <behaviortree_mtc/geometry_msgs.h>
#include <behaviortree_mtc/blackboard_helper.h>

#include <moveit/task_constructor/stages/compute_ik.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

namespace
{
constexpr auto kPortStage = "stage";
constexpr auto kPortStageName = "stage_name";
constexpr auto kPortEef = "eef";
constexpr auto kPortGroup = "group";
constexpr auto kPortWrappedStage = "wrapped_stage";
constexpr auto kPortMaxIkSolutions = "max_ik_solutions";
constexpr auto kPortIgnoreCollisions = "ignore_collisions";
constexpr auto kPortMinSolutionDistance = "min_solution_distance";
constexpr auto kPortIkFrame = "ik_frame";
}  // namespace

CreateMTCComputeIK::CreateMTCComputeIK(const std::string& name,
                                       const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreateMTCComputeIK::tick()
{
  // Retrieve inputs
  std::string name, eef, group, default_pose;
  bool ignore_collisions;
  uint32_t max_ik_solutions;
  double min_solution_distance;
  Vector3D translation_transform;
  Vector3D rotation_transform;
  MTC::StagePtr wrapped_stage;
  std::shared_ptr<geometry_msgs::PoseStamped> ik_frame{ nullptr };

  if(!getInput(kPortStageName, name) ||
     !getInput(kPortEef, eef) ||
     !getInput(kPortGroup, group) ||
     !getInput(kPortIgnoreCollisions, ignore_collisions) ||
     !getInput(kPortMinSolutionDistance, min_solution_distance) ||
     !getInput(kPortMaxIkSolutions, max_ik_solutions) ||
     !getInput(kPortIkFrame, ik_frame))
    return NodeStatus::FAILURE;

  // Transform stage from shared to unique
  auto unique_stage = convertSharedToUniqueLocked<MTC::Stage>(*this, kPortWrappedStage);

  // Build stage
  std::shared_ptr<MTC::stages::ComputeIK> wrapper{ nullptr };
  wrapper = {
    new MTC::stages::ComputeIK(name, std::move(unique_stage)),
    dirty::fake_deleter{}
  };
  wrapper->setEndEffector(eef);
  wrapper->setGroup(group);
  wrapper->setMaxIKSolutions(max_ik_solutions);
  wrapper->setIgnoreCollisions(ignore_collisions);
  wrapper->setMinSolutionDistance(min_solution_distance);
  wrapper->setIKFrame(*ik_frame);

  // Upcast to base class
  MTC::StagePtr base_stage = wrapper;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCComputeIK::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortEef, "name of end-effector"),
    BT::InputPort<std::string>(kPortStageName),
    BT::InputPort<std::string>(kPortGroup, "name of active group (derived from eef if not provided)"),
    BT::InputPort<bool>(kPortIgnoreCollisions, false, "ignore collisions, true or false"),
    BT::InputPort<uint32_t>(kPortMaxIkSolutions, 1, "max ik solutions"),
    BT::InputPort<double>(kPortMinSolutionDistance, 0.1, "minimum distance between seperate IK solutions for the same target"),
    BT::InputPort<MTC::StagePtr>(kPortWrappedStage),
    BT::InputPort<std::shared_ptr<geometry_msgs::PoseStamped>>(kPortIkFrame),

    BT::OutputPort<MTC::StagePtr>(kPortStage),
  };
}