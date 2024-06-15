#include <behaviortree_mtc/plan_mtc_task.h>

#include <moveit/task_constructor/task.h>

namespace BT {
namespace MTC {

namespace {
constexpr auto kPortTask = "task";
constexpr auto kPortMaxSolutions = "max_solutions";

}  // namespace

PlanMTCTask::PlanMTCTask(const std::string& name, const BT::NodeConfig& config)
  : ThreadedAction(name, config)
{}

BT::NodeStatus PlanMTCTask::tick()
{
  const size_t max_solutions = getInput<size_t>(kPortMaxSolutions).value();
  if(auto any_ptr = getLockedPortContent(kPortTask))
  {
    if(auto* task_ptr = any_ptr->castPtr<moveit::task_constructor::TaskPtr>())
    {
      auto& task = *task_ptr;
      if(task->plan(max_solutions))
      {
        return NodeStatus::SUCCESS;
      }
    }
  }
  return NodeStatus::FAILURE;
}

BT::PortsList PlanMTCTask::providedPorts()
{
  return {
    BT::BidirectionalPort<moveit::task_constructor::TaskPtr>(kPortTask),
    BT::InputPort<size_t>(kPortMaxSolutions),
  };
}

}  // namespace MTC
}  // namespace BT
