#include <ros/ros.h>

#include <behaviortree_mtc/initialize_mtc_task.h>
#include <behaviortree_mtc/create_mtc_current_state.h>
#include <behaviortree_mtc/move_mtc_stage_to_container.h>
#include <behaviortree_mtc/move_mtc_container_to_parent_container.h>
#include <behaviortree_mtc/create_mtc_serial_container.h>
#include <behaviortree_mtc/plan_mtc_task.h>
#include <behaviortree_mtc/moveit_msgs.h>
#include <behaviortree_mtc/geometry_msgs.h>
#include <behaviortree_mtc/create_mtc_pipeline_planner.h>
#include <behaviortree_mtc/create_planning_scene_interface.h>
#include <behaviortree_mtc/add_object_to_planning_scene.h>
#include <behaviortree_mtc/plan_mtc_task.h>
#include <behaviortree_mtc/create_mtc_generate_grasp_pose.h>
#include <behaviortree_mtc/get_mtc_raw_stage.h>
#include <behaviortree_mtc/create_mtc_move_to.h>
#include <behaviortree_mtc/create_mtc_connect.h>
#include <behaviortree_mtc/create_mtc_move_relative.h>
#include <behaviortree_mtc/create_mtc_compute_ik.h>
#include <behaviortree_mtc/configure_init_from_mtc_properties.h>
#include <behaviortree_mtc/std_containers.h>
#include <behaviortree_mtc/create_mtc_cartesian_path.h>
#include <behaviortree_mtc/create_mtc_modify_planning_scene.h>
#include <behaviortree_mtc/create_mtc_generate_place_pose.h>
#include <behaviortree_mtc/expose_mtc_properties.h>
#include <behaviortree_mtc/set_mtc_properties.h>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

using namespace BT;
using namespace BT::MTC;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_behavior_tree");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  BehaviorTreeFactory factory;

  factory.registerNodeType<GeometryMsgsPose>("GeometryMsgsPose");
  factory.registerNodeType<GeometryMsgsPoseStamped>("GeometryMsgsPoseStamped");
  factory.registerNodeType<GeometryMsgsVector3Stamped>("GeometryMsgsVector3Stamped");
  factory.registerNodeType<MoveItMsgsCollisionObjectCylinder>("MoveItMsgsCollisionObjectCylinder");
  factory.registerNodeType<MoveItMsgsCollisionObjectBox>("MoveItMsgsCollisionObjectBox");
  factory.registerNodeType<CreatePlanningSceneInterface>("CreatePlanningSceneInterface");
  factory.registerNodeType<AddObjectToPlanningScene>("AddObjectToPlanningScene");
  factory.registerNodeType<CreateMTCGenerateGraspPose>("CreateMTCGenerateGraspPose");
  factory.registerNodeType<MoveMTCStageToContainer>("MoveMTCStageToContainer");
  factory.registerNodeType<InitializeMTCTask>("InitializeMTCTask");
  factory.registerNodeType<CreateMTCCurrentState>("CreateMTCCurrentState");
  factory.registerNodeType<CreateMTCPipelinePlanner>("CreateMTCPipelinePlanner");
  factory.registerNodeType<PlanMTCTask>("PlanMTCTask");
  factory.registerNodeType<GetMTCRawStage>("GetMTCRawStage");
  factory.registerNodeType<CreateMTCMoveToNamedJointPose>("CreateMTCMoveToNamedJointPose");
  factory.registerNodeType<CreateMTCConnect>("CreateMTCConnect");
  factory.registerNodeType<MoveMTCContainerToParentContainer>("MoveMTCContainerToParentContainer");
  factory.registerNodeType<CreateMTCSerialContainer>("CreateMTCSerialContainer");
  factory.registerNodeType<CreateMTCMoveRelativeTranslate>("CreateMTCMoveRelativeTranslate");
  factory.registerNodeType<CreateMTCComputeIK>("CreateMTCComputeIK");
  factory.registerNodeType<ExposeMTCProperties<moveit::task_constructor::ContainerBase,moveit::task_constructor::ContainerBase>>("ExposeMTCContainerPropertiesToContainer");
  factory.registerNodeType<ExposeMTCProperties<moveit::task_constructor::Task,moveit::task_constructor::ContainerBase>>("ExposeMTCTaskPropertiesToContainer");
  factory.registerNodeType<ConfigureInitFromMTCProperties<moveit::task_constructor::Stage>>("StageConfigureInitFromMTCProperties");
  factory.registerNodeType<ConfigureInitFromMTCProperties<moveit::task_constructor::ContainerBase>>("ContainerConfigureInitFromMTCProperties");
  factory.registerNodeType<SetMTCProperties<moveit::task_constructor::Stage,std::string>>("SetMTCStagePropertyString");
  factory.registerNodeType<SetMTCProperties<moveit::task_constructor::Task,std::string>>("SetMTCTaskPropertyString");
  factory.registerScriptingEnums<moveit::task_constructor::Stage::PropertyInitializerSource>();
  factory.registerNodeType<CreateMTCCartesianPath>("CreateMTCCartesianPath");
  factory.registerNodeType<CreateMTCModifyPlanningSceneAttachObjects>("CreateMTCModifyPlanningSceneAttachObjects");
  factory.registerNodeType<CreateMTCModifyPlanningSceneDetachObjects>("CreateMTCModifyPlanningSceneDetachObjects");
  factory.registerNodeType<AllowCollisionPairs>("AllowCollisionPairs");
  factory.registerNodeType<AllowCollisionsJointModelGroup>("AllowCollisionsJointModelGroup");
  factory.registerNodeType<ForbidCollisionsJointModelGroup>("ForbidCollisionsJointModelGroup");
  factory.registerNodeType<ForbidCollisionPairs>("ForbidCollisionPairs");
  factory.registerNodeType<CreateMTCGeneratePlacePose>("CreateMTCGeneratePlacePose");

  auto tree = factory.createTreeFromFile("./config/bt_pick_place.xml");

  // Connect the Groot2Publisher. This will allow Groot2 to
  // get the tree and poll status updates.
  const unsigned port = 1667;
  BT::Groot2Publisher publisher(tree, port);

  // Add two more loggers, to save the transitions into a file.
  // Both formats are compatible with Groot2

  // Logging with lightweight serialization
  BT::FileLogger2 logger2(tree, "t12_logger2.btlog");

  // Gives the user time to connect to Groot2
  int wait_time = 5000;
  std::cout << "Waiting " << wait_time << " msec for connection with Groot2...\n\n"
            << std::endl;
  std::cout << "======================" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));

  std::cout << "Starting Behavior Tree" << std::endl;
  std::cout << "======================" << std::endl;

  NodeStatus status = NodeStatus::IDLE;
  while(ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
  {
    status = tree.tickExactlyOnce();
    std::cout << "Status = " << status << std::endl;

    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  std::cout << "\n\nWaiting for shutdown, press CTRL-C to quit" << std::endl;
  ros::waitForShutdown();

  return 0;
}