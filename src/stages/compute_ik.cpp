#include <behaviortree_mtc/stages/compute_ik.h>
#include <behaviortree_mtc/serialization/custom_types.h>
#include <behaviortree_mtc/msgs/geometry_msgs.h>
#include <behaviortree_mtc/bt/blackboard_helper.h>

#include <moveit/task_constructor/stages/compute_ik.h>

namespace BT {
namespace MTC {

ComputeIK::ComputeIK(const std::string& name,
                     const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus ComputeIK::tick()
{
  // Retrieve inputs
  std::string name, eef, group, default_pose;
  bool ignore_collisions;
  uint32_t max_ik_solutions;
  double min_solution_distance;
  Vector3D translation_transform;
  Vector3D rotation_transform;
  moveit::task_constructor::StagePtr wrapped_stage;
  std::shared_ptr<geometry_msgs::PoseStamped> ik_frame{ nullptr };

  if(!getInput("stage_name", name) ||
     !getInput("ignore_collisions", ignore_collisions) ||
     !getInput("min_solution_distance", min_solution_distance) ||
     !getInput("max_ik_solutions", max_ik_solutions) ||
     !getInput("ik_frame", ik_frame))
    return NodeStatus::FAILURE;

  // Transform stage: shared -> unique
  auto unique_stage = convertSharedToUniqueLocked<moveit::task_constructor::Stage>(*this, "wrapped_stage");

  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::ComputeIK> wrapper{ nullptr };
  wrapper = {
    new moveit::task_constructor::stages::ComputeIK(name, std::move(unique_stage)),
    dirty::fake_deleter{}
  };

  // Optionnal
  if(getInput("group", group))
    wrapper->setGroup(group);
  if(getInput("eef", eef))
    wrapper->setEndEffector(eef);

  wrapper->setMaxIKSolutions(max_ik_solutions);
  wrapper->setIgnoreCollisions(ignore_collisions);
  wrapper->setMinSolutionDistance(min_solution_distance);
  wrapper->setIKFrame(*ik_frame);

  // Upcast to base class
  moveit::task_constructor::StagePtr base_stage = wrapper;

  setOutput("stage", base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList ComputeIK::providedPorts()
{
  return {
    BT::InputPort<std::string>("stage_name"),
    BT::InputPort<std::string>("eef", "name of end-effector"),
    BT::InputPort<std::string>("group", "name of active group (derived from eef if not provided)"),
    BT::InputPort<bool>("ignore_collisions", false, "ignore collisions, true or false"),
    BT::InputPort<uint32_t>("max_ik_solutions", 1, "max ik solutions"),
    BT::InputPort<double>("min_solution_distance", 0.1, "minimum distance between seperate IK solutions for the same target"),
    BT::InputPort<moveit::task_constructor::StagePtr>("wrapped_stage"),
    BT::InputPort<std::shared_ptr<geometry_msgs::PoseStamped>>("ik_frame"),

    BT::OutputPort<moveit::task_constructor::StagePtr>("stage"),
  };
}

}  // namespace MTC
}  // namespace BT
