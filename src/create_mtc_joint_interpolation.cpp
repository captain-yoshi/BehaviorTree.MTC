#include <behaviortree_mtc/create_mtc_joint_interpolation.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/solvers/joint_interpolation.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

namespace
{
constexpr auto kPortSolver = "solver";
constexpr auto kPortMaxVelocityScalingFactor = "max_velocity_scaling_factor";
constexpr auto kPortMaxAccelerationScalingFactor = "max_acceleration_scaling_factor";
constexpr auto kPortMaxStep = "max_step";
}  // namespace

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
  if(!getInput(kPortMaxVelocityScalingFactor, max_velocity_scaling_factor) ||
     !getInput(kPortMaxAccelerationScalingFactor, max_acceleration_scaling_factor) ||
     !getInput(kPortMaxStep, max_step))
    return NodeStatus::FAILURE;
  
  //build solver
  auto solver = std::make_shared<MTC::solvers::JointInterpolationPlanner>();
  solver->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
  solver->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
  solver->setProperty(kPortMaxStep, max_step);

  // Upcast to base class
  MTC::solvers::PlannerInterfacePtr base_solver = solver;

  setOutput(kPortSolver, base_solver);
  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCJointInterpolation::providedPorts()
{
  return {
    //Output
    BT::OutputPort<MTC::solvers::PlannerInterfacePtr>(kPortSolver, "{solver}", "plan using simple interpolation in joint-space"),
    //Inputs
    BT::InputPort<double>(kPortMaxVelocityScalingFactor, 0.1, "scale down max velocity by this factor"),
    BT::InputPort<double>(kPortMaxAccelerationScalingFactor, 0.1, "scale down max acceleration by this factor"),
    BT::InputPort<double>(kPortMaxStep, 0.1, "max joint step"),
  };
}
