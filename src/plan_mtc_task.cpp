#include <behaviortree_mtc/plan_mtc_task.h>
#include <moveit/utils/moveit_error_code.h>

namespace BT {
namespace MTC {

PlanMTCTask::PlanMTCTask(const std::string& name, const BT::NodeConfig& config)
  : ThreadedAction(name, config)
{}

BT::NodeStatus PlanMTCTask::tick()
{
  // Inputs
  const size_t max_solutions = getInput<size_t>("max_solutions").value();
  if(auto any_ptr = getLockedPortContent("task"))
  {
    if(auto* task_ptr = any_ptr->castPtr<moveit::task_constructor::TaskPtr>())
    {
      auto& task = *task_ptr;

      // copy task internally
      {
        std::unique_lock<std::mutex> lock{ task_mutex_ };
        task_ = task;
      }
    }
  }

  if(task_)
  {
    moveit::core::MoveItErrorCode ec;
    try
    {
      task_->resetPreemptRequest();  // should not be needed
      ec = task_->plan(max_solutions);
      std::cout << moveit::core::errorCodeToString(ec) << std::endl;
    }
    catch(moveit::task_constructor::InitStageException e)
    {
      std::cout << e << std::endl;
      return NodeStatus::FAILURE;
    }

    {
      std::unique_lock<std::mutex> lock{ task_mutex_ };
      task_ = nullptr;
    }

    if(ec.val == ec.SUCCESS)
      return NodeStatus::SUCCESS;
  }

  return NodeStatus::FAILURE;
}

void PlanMTCTask::halt()
{
  {
    std::unique_lock<std::mutex> lock{ task_mutex_ };

    if(task_)
    {
      task_->preempt();
    }
  }

  // halt base class
  ThreadedAction::halt();
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
