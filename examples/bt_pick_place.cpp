#include <ros/ros.h>

#include <behaviortree_mtc/container.h>
#include <behaviortree_mtc/stage.h>
#include <behaviortree_mtc/task.h>

#include <behaviortree_mtc/moveit/planning_scene_interface.h>

#include <behaviortree_mtc/msgs/geometry_msgs.h>
#include <behaviortree_mtc/msgs/moveit_msgs.h>

#include <behaviortree_mtc/solvers/cartesian_path.h>
#include <behaviortree_mtc/solvers/joint_interpolation.h>
#include <behaviortree_mtc/solvers/pipeline_planner.h>

#include <behaviortree_mtc/serialization/std_containers.h>

#include <behaviortree_mtc/stages/compute_ik.h>
#include <behaviortree_mtc/stages/connect.h>
#include <behaviortree_mtc/stages/current_state.h>
#include <behaviortree_mtc/stages/generate_grasp_pose.h>
#include <behaviortree_mtc/stages/generate_place_pose.h>
#include <behaviortree_mtc/stages/modify_planning_scene.h>
#include <behaviortree_mtc/stages/move_relative.h>
#include <behaviortree_mtc/stages/move_to.h>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

using namespace BT;
using namespace BT::MTC;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bt_pick_place");
  ros::NodeHandle pnh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  BehaviorTreeFactory factory;

  // geometry_msgs
  factory.registerNodeType<GeometryMsgsPose>("PoseMsg");
  factory.registerNodeType<GeometryMsgsPoseStamped>("PoseStampedMsg");
  factory.registerNodeType<GeometryMsgsTwistStamped>("TwistStampedMsg");
  factory.registerNodeType<GeometryMsgsVector3Stamped>("Vector3StampedMsg");

  // moveit_msgs
  factory.registerNodeType<MoveItMsgsCollisionObjectBox>("CollisionObjectBoxMsg");
  factory.registerNodeType<MoveItMsgsCollisionObjectCylinder>("CollisionObjectCylinderMsg");

  // moveit
  factory.registerNodeType<PlanningSceneInterface>("PlanningSceneInterface");
  factory.registerNodeType<PlanningSceneInterfaceAddCollisionObject>("AddCollisionObjectToPlanningSceneInterface");

  // mtc stage/containers/task
  factory.registerNodeType<PropertiesSet<moveit::task_constructor::Stage, std::string>>("SetStagePropertiesString");
  factory.registerNodeType<PropertiesSet<moveit::task_constructor::Task, std::string>>("SetTaskPropertiesString");
  factory.registerNodeType<PropertiesExposeTo<moveit::task_constructor::ContainerBase, moveit::task_constructor::ContainerBase>>("ExposeContainerPropertiesToContainer");
  factory.registerNodeType<PropertiesExposeTo<moveit::task_constructor::Task, moveit::task_constructor::ContainerBase>>("ExposeTaskPropertiesToContainer");
  factory.registerNodeType<PropertiesConfigureInitFrom<moveit::task_constructor::Stage>>("ConfigureStagePropertiesInitFrom");
  factory.registerNodeType<PropertiesConfigureInitFrom<moveit::task_constructor::ContainerBase>>("ConfigureContainerPropertiesInitFrom");
  factory.registerNodeType<StageGetRawPtr>("GetStageRawPtr");
  factory.registerNodeType<StageMove<moveit::task_constructor::Stage, moveit::task_constructor::ContainerBase>>("MoveStageToContainer");
  factory.registerNodeType<StageMove<moveit::task_constructor::Stage, moveit::task_constructor::Task>>("MoveStageToTask");
  factory.registerNodeType<StageMove<moveit::task_constructor::ContainerBase, moveit::task_constructor::ContainerBase>>("MoveContainerToContainer");
  factory.registerNodeType<StageMove<moveit::task_constructor::ContainerBase, moveit::task_constructor::Task>>("MoveContainerToTask");
  factory.registerNodeType<SerialContainer>("SerialContainer");
  factory.registerNodeType<Task>("Task");
  factory.registerNodeType<TaskLoadRobotModel>("LoadRobotModelToTask");
  factory.registerNodeType<TaskPlan>("PlanTask");
  factory.registerScriptingEnums<moveit::task_constructor::Stage::PropertyInitializerSource>();

  // mtc solvers
  factory.registerNodeType<CartesianPath>("CartesianPath");
  factory.registerNodeType<JointInterpolation>("JointInterpolation");
  factory.registerNodeType<PipelinePlanner>("PipelinePlanner");

  // mtc stages
  factory.registerNodeType<CurrentState>("CurrentState");
  factory.registerNodeType<GenerateGraspPose>("GenerateGraspPose");
  factory.registerNodeType<GeneratePlacePose>("GeneratePlacePose");
  factory.registerNodeType<MoveToNamedJointPose>("MoveToNamedJointPose");
  factory.registerNodeType<MoveToJoint>("MoveToJoint");
  factory.registerNodeType<MoveToPoint>("MoveToPoint");
  factory.registerNodeType<MoveToPose>("MoveToPose");
  factory.registerNodeType<MoveRelativeTranslate>("MoveRelativeTranslate");
  factory.registerNodeType<MoveRelativeJoint>("MoveRelativeJoint");
  factory.registerNodeType<MoveRelativeTwist>("MoveRelativeTwist");
  factory.registerNodeType<Connect>("Connect");
  factory.registerNodeType<ComputeIK>("ComputeIK");
  factory.registerNodeType<ModifyPlanningSceneAttachObjects>("ModifyPlanningSceneAttachObjects");
  factory.registerNodeType<ModifyPlanningSceneDetachObjects>("ModifyPlanningSceneDetachObjects");
  factory.registerNodeType<ModifyPlanningSceneForbidCollisionPairs>("ModifyPlanningSceneForbidCollisionPairs");
  factory.registerNodeType<ModifyPlanningSceneAllowCollisionPairs>("ModifyPlanningSceneAllowCollisionPairs");
  factory.registerNodeType<ModifyPlanningSceneAllowAllCollisions>("ModifyPlanningSceneAllowAllCollisions");
  factory.registerNodeType<ModifyPlanningSceneForbidAllCollisions>("ModifyPlanningSceneForbidAllCollisions");
  factory.registerNodeType<ModifyPlanningSceneAllowCollisionsJointModelGroup>("ModifyPlanningSceneAllowCollisionsJointModelGroup");
  factory.registerNodeType<ModifyPlanningSceneForbidCollisionsJointModelGroup>("ModifyPlanningSceneForbidCollisionsJointModelGroup");

  std::string path = PROJECT_WS;
  Tree tree = factory.createTreeFromFile(path + "/config/bt_pick_place.xml");

  // Connect the Groot2Publisher. This will allow Groot2 to
  // get the tree and poll status updates.
  const unsigned port = 1667;
  BT::Groot2Publisher publisher(tree, port);

  // Add two more loggers, to save the transitions into a file.
  // Both formats are compatible with Groot2

  // Logging with lightweight serialization
  BT::FileLogger2 logger2(tree, "t12_logger2.btlog");

  // Gives the user time to connect to Groot2
  int delay_ms;
  pnh.param<int>("delay_ms", delay_ms, 0.0);
  if(delay_ms)
  {
    std::cout << "Waiting " << delay_ms << " msec for connection with Groot2...\n\n"
              << std::endl;
    std::cout << "======================" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
  }

  // Start ticking the Tree
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
