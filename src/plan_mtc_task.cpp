#include <behaviortree_mtc/plan_mtc_task.h>

#include <moveit/task_constructor/task.h>

namespace BT {
namespace MTC {

PlanMTCTask::PlanMTCTask(const std::string& name, const BT::NodeConfig& config)
  : ThreadedAction(name, config)
{}

BT::NodeStatus PlanMTCTask::tick()
{
  const size_t max_solutions = getInput<size_t>("max_solutions").value();
  if(auto any_ptr = getLockedPortContent("task"))
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
    BT::InputPort<size_t>("max_solutions"),
    BT::BidirectionalPort<moveit::task_constructor::TaskPtr>("task"),
  };
}

}  // namespace MTC
}  // namespace BT
