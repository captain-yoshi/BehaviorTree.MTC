#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <moveit/task_constructor/task.h>

namespace BT {
namespace MTC {

class Task : public BT::SyncActionNode
{
public:
  Task(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class TaskPlan : public BT::ThreadedAction
{
public:
  TaskPlan(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  void halt() override;
  static BT::PortsList providedPorts();

private:
  moveit::task_constructor::TaskPtr task_;

  std::mutex task_mutex_;
};

}  // namespace MTC
}  // namespace BT
