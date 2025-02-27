#include <behaviortree_mtc/solvers/create_mtc_joint_interpolation.h>
#include <behaviortree_mtc/bt/shared_to_unique.h>

#include <moveit/task_constructor/solvers/joint_interpolation.h>

namespace BT {
namespace MTC {

CreateMTCJointInterpolation::CreateMTCJointInterpolation(const std::string& name,
                                                         const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreateMTCJointInterpolation::tick()
{
  // Retrieve inputs
  double max_velocity_scaling_factor;
  double max_acceleration_scaling_factor;
  double max_step;

  //Inputs
  if(!getInput("max_velocity_scaling_factor", max_velocity_scaling_factor) ||
     !getInput("max_acceleration_scaling_factor", max_acceleration_scaling_factor) ||
     !getInput("max_step", max_step))
    return NodeStatus::FAILURE;

  //build solver
  auto solver = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
  solver->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
  solver->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
  solver->setProperty("max_step", max_step);

  // Upcast to base class
  moveit::task_constructor::solvers::PlannerInterfacePtr base_solver = solver;

  setOutput("solver", base_solver);
  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCJointInterpolation::providedPorts()
{
  return {
    BT::InputPort<double>("max_velocity_scaling_factor", 0.1, "scale down max velocity by this factor"),
    BT::InputPort<double>("max_acceleration_scaling_factor", 0.1, "scale down max acceleration by this factor"),
    BT::InputPort<double>("max_step", 0.1, "max joint step"),
    BT::OutputPort<moveit::task_constructor::solvers::PlannerInterfacePtr>("solver", "{solver}", "plan using simple interpolation in joint-space"),
  };
}

}  // namespace MTC
}  // namespace BT
