#pragma once

<<<<<<< HEAD
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <behaviortree_cpp/bt_factory.h>

namespace MTC = moveit::task_constructor;
namespace bt_mtc{
class CreatePipelinePlanner : public BT::SyncActionNode
{
public:
  CreatePipelinePlanner(const std::string& name, const BT::NodeConfig& config);
  void setPipelineProperties(std::string property_name,std::string type, std::shared_ptr<MTC::solvers::PipelinePlanner> pipelinePlanner);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};
}

=======
#include <behaviortree_cpp/bt_factory.h>

namespace bt_mtc
{
class CreateMTCPipelinePlanner : public BT::SyncActionNode
{
public:
  CreateMTCPipelinePlanner(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};
}  // namespace bt_mtc
>>>>>>> [impl] pipeline planner solver
