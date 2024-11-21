#include <ros/ros.h>

#include <behaviortree_mtc/initialize_mtc_task.h>
#include <behaviortree_mtc/create_mtc_current_state.h>
#include <behaviortree_mtc/move_mtc_stage.h>
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

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

using namespace BT;
using namespace BT::MTC;

// clang-format off
static const char* xml_text = R"(

 <root BTCPP_format="4" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <Script code="object_name:='object' " />
            <Script code="table_name:='table' " />
            <Script code="reference_frame:='world' " />
            <Script code="eef:='hand' " />
            <Script code="angle:=3.141592653589793/12 " />
            <Script code="arm_group_name:='panda_arm'" />
            <Script code="hand_group_name:='hand'" />
            <Script code="hand_frame:='panda_link8'" />
            <CreatePlanningSceneInterface  planning_scene_interface="{psi}" />
            <GeometryMsgsPose  name="box's pose"
                               position="0.5,-0.25,-0.05"
                               quaternion="1,0,0,0"
                               pose="{box_pose}" />
            <MoveItMsgsCollisionObjectBox object_id="{table_name}"
                                          length="0.4"
                                          width="0.5"
                                          height="0.1"
                                          pose="{box_pose}"
                                          frame_id="{reference_frame}"
                                          collision_object="{box}"  />
            <AddObjectToPlanningScene   planning_scene_interface="{psi}"
                          collision_object="{box}"  />
            <GeometryMsgsPose  name="cylinder's pose"
                               position="0.5,-0.25,0.125"
                               quaternion="1,0,0,0"
                               pose="{cylinder_pose}" />
            <MoveItMsgsCollisionObjectCylinder  object_id="{object_name}"
                                                radius="0.02"
                                                height="0.25"
                                                pose="{cylinder_pose}"
                                                frame_id="{reference_frame}"
                                                collision_object="{cylinder}"  />
            <AddObjectToPlanningScene  planning_scene_interface="{psi}" 
                         collision_object="{cylinder}"/> 
            <InitializeMTCTask        task="{mtc_task}" />
            <CreateMTCPipelinePlanner pipeline_id="ompl"
                                 planner_id="RRTConnect"
                                 solver="{rrt_connect}"
                                 goal_joint_tolerance="1e-5" />
            <CreateMTCCurrentState    stage="{current_state}" />
            <MoveMTCStageToTask  child="{current_state}" parent="{mtc_task}" />
            <GeometryMsgsPoseStamped  frame_id="{hand_frame}" position="0,0,0" quaternion="1,0,0,0" pose_stamped="{ik_frame}"/>
            <CreateMTCMoveToNamedJointPose name="move to -> close_hand motion"
                                      group="{hand_group_name}"
                                      solver="{rrt_connect}"
                                      goal="close"
                                      stage="{stage_move_to_close_hand}" />
            <MoveMTCStageToTask  child="{stage_move_to_close_hand}" parent="{mtc_task}" />
            <CreateMTCMoveToNamedJointPose name="move to -> open_hand motion"
                                      group="{hand_group_name}"
                                      solver="{rrt_connect}"
                                      goal="open"
                                      stage="{stage_move_to_open_hand}" />
            <GetMTCRawStage  stage="{stage_move_to_open_hand}" raw_stage="{raw_stage}" />
            <MoveMTCStageToTask  child="{stage_move_to_open_hand}" parent="{mtc_task}" />
            <CreateMTCConnect  stage_name="MoveToPick"
                               timeout="5.0"
                               group="{arm_group_name}"
                               planner="{rrt_connect}"
                               stage="{connect}" />
            <MoveMTCStageToTask  child="{connect}" parent="{mtc_task}" />
            <CreateMTCSerialContainer  container_name="pick object"
                                       container="{pick_object}" />
            <GeometryMsgsVector3Stamped  frame_id="{hand_frame}" vector="0,0,1" vector3_stamped="{tcp_translate}"/>
            <CreateMTCMoveRelativeTranslate  stage_name="approach object"
                                             stage="{approach_object}"
                                             group="{arm_group_name}"
                                             solver="{rrt_connect}"
                                             ik_frame="{ik_frame}"
                                             direction="{tcp_translate}"
                                             min_distance="0.1"
                                             max_distance="0.15" />
            <MoveMTCStageToContainer  child="{approach_object}" parent="{pick_object}" />
            <CreateMTCGenerateGraspPose  stage_name="generate_grasp_pose"
                                         eef="{eef}"
                                         object="{object_name}"
                                         angle_delta="{angle}"
                                         pregrasp_pose="open"
                                         monitored_stage="{raw_stage}"
                                         stage="{generate_pose}" />
            <GeometryMsgsPoseStamped  frame_id="{hand_frame}" position="0,0,0.1" quaternion="0.270595,0.653228,-0.270861,0.653228" pose_stamped="{ik_frame2}"/>
            <CreateMTCComputeIK  stage_name="grasp pose IK"
                                      eef="{eef}"
                                      group="{arm_group_name}"
                                      wrapped_stage="{generate_pose}"
                                      max_ik_solutions="8"
                                      min_solution_distance="1.0"
                                      stage="{grasp_pose_IK}"
                                      ik_frame="{ik_frame2}"  />
            <ConfigureInitFromMTCProperties  stage="{grasp_pose_IK}"
                                             property_names="target_pose"
                                             source_flag="INTERFACE" />
            <MoveMTCStageToContainer child="{grasp_pose_IK}" parent="{pick_object}" />
            <MoveMTCContainerToTask  child="{pick_object}" parent="{mtc_task}" />
            <PlanMTCTask              task="{mtc_task}" max_solutions="5" />
        </Sequence>
     </BehaviorTree>

 </root>
 )";
// clang-format on

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bt_mtc_demo");
  ros::NodeHandle pnh("~");

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
  factory.registerNodeType<InitializeMTCTask>("InitializeMTCTask");
  factory.registerNodeType<CreateMTCCurrentState>("CreateMTCCurrentState");
  factory.registerNodeType<CreateMTCPipelinePlanner>("CreateMTCPipelinePlanner");
  factory.registerNodeType<PlanMTCTask>("PlanMTCTask");
  factory.registerNodeType<GetMTCRawStage>("GetMTCRawStage");
  factory.registerNodeType<CreateMTCMoveToNamedJointPose>("CreateMTCMoveToNamedJointPose");
  factory.registerNodeType<CreateMTCConnect>("CreateMTCConnect");
  factory.registerNodeType<MoveMTCStage<moveit::task_constructor::Stage, moveit::task_constructor::ContainerBase>>("MoveMTCStageToContainer");
  factory.registerNodeType<MoveMTCStage<moveit::task_constructor::Stage, moveit::task_constructor::Task>>("MoveMTCStageToTask");
  factory.registerNodeType<MoveMTCStage<moveit::task_constructor::ContainerBase, moveit::task_constructor::Task>>("MoveMTCContainerToTask");
  factory.registerNodeType<CreateMTCSerialContainer>("CreateMTCSerialContainer");
  factory.registerNodeType<CreateMTCMoveRelativeTranslate>("CreateMTCMoveRelativeTranslate");
  factory.registerNodeType<CreateMTCComputeIK>("CreateMTCComputeIK");
  factory.registerNodeType<ConfigureInitFromMTCProperties<moveit::task_constructor::Stage>>("ConfigureInitFromMTCProperties");
  factory.registerScriptingEnums<moveit::task_constructor::Stage::PropertyInitializerSource>();

  auto tree = factory.createTreeFromText(xml_text);

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
