#include <behaviortree_mtc/create_mtc_cartesian_path.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

namespace BT {
namespace MTC {

CreateMTCCartesianPath::CreateMTCCartesianPath(const std::string& name,
                                               const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreateMTCCartesianPath::tick()
{
  // Retrieve inputs
  double max_velocity_scaling_factor;
  double max_acceleration_scaling_factor;
  double step_size;
  double jump_treshold;
  double min_fraction;

  if(!getInput("max_velocity_scaling_factor", max_velocity_scaling_factor) ||
     !getInput("max_acceleration_scaling_factor", max_acceleration_scaling_factor) ||
     !getInput("step_size", step_size) ||
     !getInput("min_fraction", min_fraction) ||
     !getInput("jump_threshold", jump_treshold))
    return NodeStatus::FAILURE;

  //build solver
  auto solver = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
  solver->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
  solver->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
  solver->setStepSize(step_size);
  solver->setJumpThreshold(jump_treshold);
  solver->setMinFraction(min_fraction);
  // Upcast to base class
  moveit::task_constructor::solvers::PlannerInterfacePtr base_solver = solver;

  setOutput("solver", base_solver);
  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCCartesianPath::providedPorts()
{
  return {
    BT::InputPort<double>("max_velocity_scaling_factor", 0.1, "scale down max velocity by this factor"),
    BT::InputPort<double>("max_acceleration_scaling_factor", 0.1, "scale down max acceleration by this factor"),
    BT::InputPort<double>("step_size", 0.01, "step size between consecutive waypoints"),
    BT::InputPort<double>("min_fraction", 1.0, "fraction of motion required for success"),
    BT::InputPort<double>("jump_threshold", 1.5, "acceptable fraction of mean joint motion per step"),
    BT::OutputPort<moveit::task_constructor::solvers::PlannerInterfacePtr>("solver", "{solver}", "Planner interface using pipeline motion solver"),
  };
}

}  // namespace MTC
}  // namespace BT
