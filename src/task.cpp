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

TaskLoadRobotModel::TaskLoadRobotModel(const std::string& name, const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus TaskLoadRobotModel::tick()
{
  std::string robot_description;

  if(!getInput("robot_description", robot_description))
    return NodeStatus::FAILURE;

  if(auto any_ptr = getLockedPortContent("task"))
  {
    if(auto* task_ptr = any_ptr->castPtr<moveit::task_constructor::TaskPtr>())
    {
      auto& task = *task_ptr;

      task->loadRobotModel(robot_description);
      return NodeStatus::SUCCESS;
    }
  }

  return NodeStatus::FAILURE;
}

BT::PortsList TaskLoadRobotModel::providedPorts()
{
  return {
    BT::InputPort<std::string>("robot_description", "robot_description"),
    BT::BidirectionalPort<moveit::task_constructor::TaskPtr>("task"),
  };
}

TaskGetRobotModel::TaskGetRobotModel(const std::string& name, const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus TaskGetRobotModel::tick()
{
  moveit::core::RobotModelPtr robot_model;

  if(auto any_ptr = getLockedPortContent("task"))
  {
    if(auto* task_ptr = any_ptr->castPtr<moveit::task_constructor::TaskPtr>())
    {
      auto& task = *task_ptr;

      setOutput("robot_model", task->getRobotModel());
      return NodeStatus::SUCCESS;
    }
  }

  return NodeStatus::FAILURE;
}

BT::PortsList TaskGetRobotModel::providedPorts()
{
  return {
    BT::BidirectionalPort<moveit::task_constructor::TaskPtr>("task"),
    BT::OutputPort<moveit::core::RobotModelConstPtr>("robot_model"),
  };
}

TaskSetRobotModel::TaskSetRobotModel(const std::string& name, const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus TaskSetRobotModel::tick()
{
  moveit::core::RobotModelPtr robot_model;

  if(!getInput("robot_model", robot_model))
    return NodeStatus::FAILURE;

  if(auto any_ptr = getLockedPortContent("task"))
  {
    if(auto* task_ptr = any_ptr->castPtr<moveit::task_constructor::TaskPtr>())
    {
      auto& task = *task_ptr;

      task->setRobotModel(robot_model);
      return NodeStatus::SUCCESS;
    }
  }

  return NodeStatus::FAILURE;
}

BT::PortsList TaskSetRobotModel::providedPorts()
{
  return {
    BT::InputPort<moveit::core::RobotModelConstPtr>("robot_model"),
    BT::BidirectionalPort<moveit::task_constructor::TaskPtr>("task"),
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
