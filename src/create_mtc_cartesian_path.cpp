#include <behaviortree_mtc/create_mtc_cartesian_path.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

namespace BT {
namespace MTC {

namespace {
constexpr auto kPortSolver = "solver";
constexpr auto kPortMaxVelocityScalingFactor = "max_velocity_scaling_factor";
constexpr auto kPortMaxAccelerationScalingFactor = "max_acceleration_scaling_factor";
constexpr auto kPortStepSize = "step_size";
constexpr auto kPortJumpTreshold = "jump_threshold";
constexpr auto kPortMinFraction = "min_fraction";
}  // namespace

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

  //Inputs
  if(!getInput(kPortMaxVelocityScalingFactor, max_velocity_scaling_factor) ||
     !getInput(kPortMaxAccelerationScalingFactor, max_acceleration_scaling_factor) ||
     !getInput(kPortStepSize, step_size) ||
     !getInput(kPortMinFraction, min_fraction) ||
     !getInput(kPortJumpTreshold, jump_treshold))
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

  setOutput(kPortSolver, base_solver);
  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCCartesianPath::providedPorts()
{
  return {
    //Output
    BT::OutputPort<moveit::task_constructor::solvers::PlannerInterfacePtr>(kPortSolver, "{solver}", "Planner interface using pipeline motion solver"),
    //Inputs
    BT::InputPort<double>(kPortMaxVelocityScalingFactor, 0.1, "scale down max velocity by this factor"),
    BT::InputPort<double>(kPortMaxAccelerationScalingFactor, 0.1, "scale down max acceleration by this factor"),
    BT::InputPort<double>(kPortStepSize, 0.01, "step size between consecutive waypoints"),
    BT::InputPort<double>(kPortJumpTreshold, 1.5, "acceptable fraction of mean joint motion per step"),
    BT::InputPort<double>(kPortMinFraction, 1.0, "fraction of motion required for success"),
  };
}

}  // namespace MTC
}  // namespace BT
