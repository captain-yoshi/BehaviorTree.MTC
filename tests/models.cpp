#include "models.h"
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>

using namespace moveit::core;

RobotModelPtr getModel()
{
  // suppress RobotModel errors and warnings
  rclcpp::Logger logger = rclcpp::get_logger("moveit_core.robot_model");
  logger.set_level(rclcpp::Logger::Level::Fatal);

  // create dummy robot model
  moveit::core::RobotModelBuilder builder("robot", "base");
  builder.addChain("base->link1->link2->tip", "continuous");
  builder.addGroupChain("base", "link2", "group");
  builder.addGroupChain("link2", "tip", "eef_group");
  builder.addEndEffector("eef", "link2", "group", "eef_group");
  return builder.build();
}

moveit::core::RobotModelPtr loadModel()
{
  auto node = rclcpp::Node::make_shared("robot_model_loader_node");
  robot_model_loader::RobotModelLoader::Options options;
  robot_model_loader::RobotModelLoader loader(node, options);
  return loader.getModel();
}
