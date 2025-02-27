#include <behaviortree_mtc/solvers/create_mtc_cartesian_path.h>
#include <behaviortree_mtc/bt/shared_to_unique.h>

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
  double min_fraction;
  moveit::core::CartesianPrecision precision;

  if(!getInput("max_velocity_scaling_factor", max_velocity_scaling_factor) ||
     !getInput("max_acceleration_scaling_factor", max_acceleration_scaling_factor) ||
     !getInput("step_size", step_size) ||
     !getInput("min_fraction", min_fraction) ||
     !getInput("precision", precision))
    return NodeStatus::FAILURE;

  // Build solver
  auto solver = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
  solver->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
  solver->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
  solver->setStepSize(step_size);
  solver->setMinFraction(min_fraction);
  solver->setPrecision(precision);

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
    BT::InputPort<double>("min_fraction", 1.0, "fraction of motion required for success"),
    BT::InputPort<moveit::core::CartesianPrecision>("precision", moveit::core::CartesianPrecision({ .translational = 0.001, .rotational = 0.01, .max_resolution = 1e-5 }), "linear and rotational precision"),
    BT::InputPort<double>("step_size", 0.01, "step size between consecutive waypoints"),
    BT::OutputPort<moveit::task_constructor::solvers::PlannerInterfacePtr>("solver", "{solver}", "Planner interface using pipeline motion solver"),
  };
}

}  // namespace MTC
}  // namespace BT
