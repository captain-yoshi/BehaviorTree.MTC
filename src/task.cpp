#include <behaviortree_mtc/task.h>

#include <moveit/task_constructor/task.h>

namespace BT {
namespace MTC {

Task::Task(const std::string& name, const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus Task::tick()
{
  auto task = std::make_shared<moveit::task_constructor::Task>();
  task->loadRobotModel();

  setOutput("task", task);

  return NodeStatus::SUCCESS;
}

BT::PortsList Task::providedPorts()
{
  return {
    BT::OutputPort<moveit::task_constructor::TaskPtr>("task", "{mtc_task}",
                                                      "MoveIt Task Constructor task."),
  };
}

TaskPlan::TaskPlan(const std::string& name, const BT::NodeConfig& config)
  : ThreadedAction(name, config)
{}

BT::NodeStatus TaskPlan::tick()
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
      std::cout << ec.toString(ec) << std::endl;
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

void TaskPlan::halt()
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

BT::PortsList TaskPlan::providedPorts()
{
  return {
    BT::InputPort<size_t>("max_solutions"),
    BT::BidirectionalPort<moveit::task_constructor::TaskPtr>("task"),
  };
}

}  // namespace MTC
}  // namespace BT
