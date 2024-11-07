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

// clang-format off

static const char* xml_text = R"(

 <root BTCPP_format="4" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root">

            <!-- Set Parameters -->
            <Script code="object_name:='object' " />
            <Script code="table_name:='table' " />
            <Script code="reference_frame:='world' " />
            <Script code="eef:='hand' " />
            <Script code="angle:=3.141592653589793/12 " />
            <Script code="arm_group_name:='panda_arm'" />
            <Script code="hand_group_name:='hand'" />
            <Script code="hand_frame:='panda_link8'" />

            <!-- PlanningSceneInterface -->
            <CreatePlanningSceneInterface  planning_scene_interface="{psi}" />

            <!-- Create Table -->
            <GeometryMsgsPose name="box's pose" position="0.5,-0.25,-0.05" quaternion="1,0,0,0" pose="{box_pose}" />
            <MoveItMsgsCollisionObjectBox object_id="{table_name}" length="0.4" width="0.5" height="0.1" pose="{box_pose}" frame_id="{reference_frame}" collision_object="{box}" />
            <AddObjectToPlanningScene planning_scene_interface="{psi}" collision_object="{box}" />

            <!-- Create Cylinder -->
            <GeometryMsgsPose name="cylinder's pose" position="0.5,-0.25,0.125" quaternion="1,0,0,0" pose="{cylinder_pose}" />
            <MoveItMsgsCollisionObjectCylinder object_id="{object_name}"
                                               radius="0.02"
                                               height="0.25"
                                               pose="{cylinder_pose}"
                                               frame_id="{reference_frame}"
                                               collision_object="{cylinder}" />
            <AddObjectToPlanningScene planning_scene_interface="{psi}" collision_object="{cylinder}" />

            <!-- Create Task -->
            <InitializeMTCTask task="{mtc_task}" />

            <!-- Add properpties to the Task -->
            <SetMTCTaskPropertyString  stage="{mtc_task}"  property_name="eef"  property="{eef}" />
            <SetMTCTaskPropertyString  stage="{mtc_task}"  property_name="group"  property="{arm_group_name}" />
            <SetMTCTaskPropertyString  stage="{mtc_task}"  property_name="hand"  property="{hand_group_name}" />
            <SetMTCTaskPropertyString  stage="{mtc_task}"  property_name="hand_grasping_frame"  property="{hand_frame}" />
            <SetMTCTaskPropertyString  stage="{mtc_task}"  property_name="ik_frame"  property="{hand_frame}" />

            <!-- Create Solvers -->
            <CreateMTCPipelinePlanner pipeline_id="ompl" planner_id="RRTConnect" solver="{rrt_connect}" goal_joint_tolerance="1e-5" />
            <CreateMTCCartesianPath solver="{cartesian}" max_velocity_scaling_factor="1.0" max_acceleration_scaling_factor="1.0" />

            <!-- Current State -->
            <CreateMTCCurrentState stage="{current_state}" />
            <MoveMTCStageToTask child="{current_state}" parent="{mtc_task}" />


            <!-- Open Hand -->
            <GeometryMsgsPoseStamped  frame_id="{hand_frame}" position="0,0,0" quaternion="1,0,0,0" pose_stamped="{ik_frame}" />
            <CreateMTCMoveToNamedJointPose name="move to -> open_hand motion"
                                           group="{hand_group_name}"
                                           solver="{rrt_connect}"
                                           goal="open"
                                           stage="{stage_move_to_open_hand}" />
            <GetMTCRawStage stage="{stage_move_to_open_hand}" raw_stage="{raw_stage}" />
            <MoveMTCStageToTask child="{stage_move_to_open_hand}" parent="{mtc_task}" />


            <!-- Move to Pick -->
            <CreateMTCConnect stage_name="MoveToPick"
                              timeout="5.0"
                              group="{arm_group_name}"
                              planner="{rrt_connect}"
                              stage="{connect}" />
            <StageConfigureInitFromMTCProperties stage="{connect}" property_names="" source_flag="PARENT" />
            <MoveMTCStageToTask child="{connect}" parent="{mtc_task}" />

            <!-- Pick Object-->
            <CreateMTCSerialContainer container_name="pick object" container="{pick_object}" />
            <ExposeMTCTaskPropertiesToContainer from="{mtc_task}" to="{pick_object}" property_names="group,eef,hand,ik_frame" />
            <ContainerConfigureInitFromMTCProperties stage="{pick_object}" property_names="group,eef,hand,ik_frame" source_flag="PARENT" />

                <!-- Approach Object -->
                <GeometryMsgsVector3Stamped frame_id="{hand_frame}" vector="0,0,1" vector3_stamped="{tcp_translate}"/>
                <CreateMTCMoveRelativeTranslate stage_name="approach object"
                                                stage="{approach_object}"
                                                solver="{rrt_connect}"
                                                ik_frame="{ik_frame}"
                                                direction="{tcp_translate}"
                                                min_distance="0.1"
                                                max_distance="0.15" />
                <StageConfigureInitFromMTCProperties stage="{approach_object}" property_names="group" source_flag="PARENT" />
                <MoveMTCStageToContainer child="{approach_object}" parent="{pick_object}" />

                <!-- Generate Grasp Pose -->
                <CreateMTCGenerateGraspPose stage_name="generate_grasp_pose"
                                            object="{object_name}"
                                            angle_delta="{angle}"
                                            pregrasp_pose="open"
                                            monitored_stage="{raw_stage}"
                                            stage="{generate_pose}" />
                <StageConfigureInitFromMTCProperties stage="{generate_pose}" property_names="" source_flag="PARENT" />
                <GeometryMsgsPoseStamped frame_id="{hand_frame}" position="0,0,0.1" quaternion="0.270595,0.653228,-0.270861,0.653228" pose_stamped="{ik_frame2}"/>
                <CreateMTCComputeIK stage_name="grasp pose IK"
                                    wrapped_stage="{generate_pose}"
                                    max_ik_solutions="8"
                                    min_solution_distance="1.0"
                                    stage="{grasp_pose_IK}"
                                    ik_frame="{ik_frame2}"  />
                <StageConfigureInitFromMTCProperties  stage="{grasp_pose_IK}" property_names="eef,group" source_flag="PARENT" />
                <StageConfigureInitFromMTCProperties  stage="{grasp_pose_IK}" property_names="target_pose" source_flag="INTERFACE" />
                <MoveMTCStageToContainer child="{grasp_pose_IK}" parent="{pick_object}" />

                <!-- Allow Collisions (hand object) -->
                <AllowCollisionsJointModelGroup stage_name="allow collisions -> hand,object"
                                                            stage="{allow_collisions_object_robot}"
                                                            objects="{object_name}"
                                                            jmg_name="{hand_group_name}"
                                                            task="{mtc_task}" />
                <MoveMTCStageToContainer child="{allow_collisions_object_robot}" parent="{pick_object}" />

                <!-- Close Hand -->
                <CreateMTCMoveToNamedJointPose name="close hand"
                                               group="{hand_group_name}"
                                               solver="{rrt_connect}"
                                               goal="close"
                                               stage="{stage_move_to_close_hand}" />
                <MoveMTCStageToContainer child="{stage_move_to_close_hand}" parent="{pick_object}" />

                <!-- Attach Object -->
                <CreateMTCModifyPlanningSceneAttachObjects stage_name="attach object"
                                                           stage="{attach_object}"
                                                           object_names="{object_name}"
                                                           link_name="{hand_frame}" />
                <MoveMTCStageToContainer child="{attach_object}" parent="{pick_object}" />

                <!-- Allow Collision (object support) -->
                <AllowCollisionPairs stage_name="allow collisions -> object,support"
                                     stage="{allow_collisions_table}"
                                     first="{object_name}"
                                     second="table" />
                <MoveMTCStageToContainer child="{allow_collisions_table}" parent="{pick_object}" />

                <!-- Lift Object -->
                <GeometryMsgsVector3Stamped frame_id="{reference_frame}" vector="0,0,1" vector3_stamped="{tcp_translate}"/>
                <CreateMTCMoveRelativeTranslate stage_name="lift object"
                                                stage="{lift_object}"
                                                solver="{cartesian}"
                                                ik_frame="{ik_frame}"
                                                direction="{tcp_translate}"
                                                min_distance="0.01"
                                                max_distance="0.1" />
                <StageConfigureInitFromMTCProperties stage="{lift_object}" property_names="group" source_flag="PARENT" />
                <MoveMTCStageToContainer child="{lift_object}" parent="{pick_object}" />

                <!-- Forbid Collision (object support) -->
                <ForbidCollisionPairs  stage_name="forbid collisions -> object,support"
                                       stage="{forbid_collisions_table}"
                                       first="{object_name}"
                                       second="table" />
                <GetMTCRawStage  stage="{forbid_collisions_table}" raw_stage="{raw_forbid_collisions_table}"  />
                <MoveMTCStageToContainer child="{forbid_collisions_table}" parent="{pick_object}" />

                <!-- Add 'pick_object' Container to Task -->
                <MoveMTCContainerToTask child="{pick_object}" parent="{mtc_task}" />

            <!-- Move to Place -->
            <CreateMTCConnect stage_name="Move To Place"
                              timeout="5.0"
                              group="{arm_group_name}"
                              planner="{rrt_connect}"
                              stage="{connectPlace}" />
            <StageConfigureInitFromMTCProperties stage="{connectPlace}" property_names="" source_flag="PARENT" />
            <MoveMTCStageToTask child="{connectPlace}" parent="{mtc_task}" />

            <!-- Place Object -->
            <CreateMTCSerialContainer container_name="place object" container="{place_object}" />
            <ExposeMTCTaskPropertiesToContainer from="{mtc_task}" to="{place_object}" property_names="group,eef,hand" />
            <ContainerConfigureInitFromMTCProperties stage="{place_object}" property_names="group,eef,hand" source_flag="PARENT" />


                <!-- Lower Object -->
                <GeometryMsgsVector3Stamped frame_id="{reference_frame}" vector="0,0,-1" vector3_stamped="{tcp_translate2}" />
                <CreateMTCMoveRelativeTranslate stage_name="lower object"
                                                stage="{lower_object}"
                                                solver="{cartesian}"
                                                ik_frame="{ik_frame}"
                                                direction="{tcp_translate2}"
                                                min_distance="0.03"
                                                max_distance="0.13" />
                <StageConfigureInitFromMTCProperties stage="{lower_object}" property_names="group" source_flag="PARENT" />
                <MoveMTCStageToContainer child="{lower_object}" parent="{place_object}" />

                <!-- Generate Place Pose -->
                <GeometryMsgsPoseStamped frame_id="world" position="0.6,-0.15,0.1251" quaternion="1,0,0,0" pose_stamped="{place_pose}" />
                <CreateMTCGeneratePlacePose stage="{generate_place_pose}"
                                            stage_name="generate place pose"
                                            object="{object_name}"
                                            pose="{place_pose}"
                                            monitored_stage="{raw_forbid_collisions_table}" />
                <CreateMTCComputeIK stage_name="place pose IK"
                                    wrapped_stage="{generate_place_pose}"
                                    max_ik_solutions="2"
                                    stage="{place_pose_IK}"
                                    ik_frame="{ik_frame2}"  />
                <StageConfigureInitFromMTCProperties  stage="{place_pose_IK}" property_names="eef,group" source_flag="PARENT" />
                <StageConfigureInitFromMTCProperties  stage="{place_pose_IK}" property_names="target_pose" source_flag="INTERFACE" />
                <MoveMTCStageToContainer child="{place_pose_IK}" parent="{place_object}" />

                <!-- Open Hand -->
                <CreateMTCMoveToNamedJointPose name="move to -> open_hand motion"
                                          group="{hand_group_name}"
                                          solver="{rrt_connect}"
                                          goal="open"
                                          stage="{stage_move_to_open_hand2}" />
                <MoveMTCStageToContainer child="{stage_move_to_open_hand2}" parent="{place_object}" />

                <!-- Forbid Collision (hand, object) -->
                <ForbidCollisionsJointModelGroup stage_name="forbid collisions -> hand,object"
                                                 stage="{forbid_collisions_object_robot}"
                                                 objects="{object_name}"
                                                 jmg_name="{hand_group_name}"
                                                 task="{mtc_task}" />
                <MoveMTCStageToContainer child="{forbid_collisions_object_robot}" parent="{place_object}" />

                <!-- Detach Object -->
                <CreateMTCModifyPlanningSceneDetachObjects stage_name="detach object"
                                                           stage="{detach_object}"
                                                           object_names="{object_name}"
                                                           link_name="{hand_frame}" />
                <MoveMTCStageToContainer child="{detach_object}" parent="{place_object}" />

                <!-- Retreat Motion -->
                <GeometryMsgsVector3Stamped frame_id="{hand_frame}" vector="0,0,-1" vector3_stamped="{tcp_translate3}" />
                <CreateMTCMoveRelativeTranslate stage_name="retreat after place"
                                                stage="{retreat_after_place}"
                                                solver="{cartesian}"
                                                ik_frame="{ik_frame}"
                                                direction="{tcp_translate3}"
                                                min_distance="0.12"
                                                max_distance="0.25" />
                <StageConfigureInitFromMTCProperties stage="{retreat_after_place}" property_names="group" source_flag="PARENT" />
                <MoveMTCStageToContainer child="{retreat_after_place}" parent="{place_object}" />

                <!-- Add 'place_object' Container to Task -->
                <MoveMTCContainerToTask child="{place_object}" parent="{mtc_task}" />

            <!-- Move to Home -->
            <CreateMTCMoveToNamedJointPose name="move home"
                                           solver="{rrt_connect}"
                                           goal="ready"
                                           stage="{stage_move_to_home}" />
            <StageConfigureInitFromMTCProperties stage="{stage_move_to_home}" property_names="group" source_flag="PARENT" />
            <MoveMTCStageToTask child="{stage_move_to_home}" parent="{mtc_task}" />

            <!-- Plan the Task -->
            <PlanMTCTask task="{mtc_task}" max_solutions="5" />
        </Sequence>
     </BehaviorTree>

 </root>
 )";

// clang-format on

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
  factory.registerNodeType<InitializeMTCTask>("InitializeMTCTask");
  factory.registerNodeType<CreateMTCCurrentState>("CreateMTCCurrentState");
  factory.registerNodeType<CreateMTCPipelinePlanner>("CreateMTCPipelinePlanner");
  factory.registerNodeType<PlanMTCTask>("PlanMTCTask");
  factory.registerNodeType<GetMTCRawStage>("GetMTCRawStage");
  factory.registerNodeType<CreateMTCMoveToNamedJointPose>("CreateMTCMoveToNamedJointPose");
  factory.registerNodeType<CreateMTCConnect>("CreateMTCConnect");
  factory.registerNodeType<MoveMTCStage<moveit::task_constructor::Stage, moveit::task_constructor::ContainerBase>>("MoveMTCStageToContainer");
  factory.registerNodeType<MoveMTCStage<moveit::task_constructor::Stage, moveit::task_constructor::Task>>("MoveMTCStageToTask");
  factory.registerNodeType<MoveMTCStage<moveit::task_constructor::ContainerBase, moveit::task_constructor::ContainerBase>>("MoveMTCContainerToContainer");
  factory.registerNodeType<MoveMTCStage<moveit::task_constructor::ContainerBase, moveit::task_constructor::Task>>("MoveMTCContainerToTask");
  factory.registerNodeType<CreateMTCSerialContainer>("CreateMTCSerialContainer");
  factory.registerNodeType<CreateMTCMoveRelativeTranslate>("CreateMTCMoveRelativeTranslate");
  factory.registerNodeType<CreateMTCComputeIK>("CreateMTCComputeIK");
  factory.registerNodeType<ExposeMTCProperties<moveit::task_constructor::ContainerBase, moveit::task_constructor::ContainerBase>>("ExposeMTCContainerPropertiesToContainer");
  factory.registerNodeType<ExposeMTCProperties<moveit::task_constructor::Task, moveit::task_constructor::ContainerBase>>("ExposeMTCTaskPropertiesToContainer");
  factory.registerNodeType<ConfigureInitFromMTCProperties<moveit::task_constructor::Stage>>("StageConfigureInitFromMTCProperties");
  factory.registerNodeType<ConfigureInitFromMTCProperties<moveit::task_constructor::ContainerBase>>("ContainerConfigureInitFromMTCProperties");
  factory.registerNodeType<SetMTCProperties<moveit::task_constructor::Stage, std::string>>("SetMTCStagePropertyString");
  factory.registerNodeType<SetMTCProperties<moveit::task_constructor::Task, std::string>>("SetMTCTaskPropertyString");
  factory.registerScriptingEnums<moveit::task_constructor::Stage::PropertyInitializerSource>();
  factory.registerNodeType<CreateMTCCartesianPath>("CreateMTCCartesianPath");
  factory.registerNodeType<CreateMTCModifyPlanningSceneAttachObjects>("CreateMTCModifyPlanningSceneAttachObjects");
  factory.registerNodeType<CreateMTCModifyPlanningSceneDetachObjects>("CreateMTCModifyPlanningSceneDetachObjects");
  factory.registerNodeType<AllowCollisionPairs>("AllowCollisionPairs");
  factory.registerNodeType<AllowCollisionsJointModelGroup>("AllowCollisionsJointModelGroup");
  factory.registerNodeType<ForbidCollisionsJointModelGroup>("ForbidCollisionsJointModelGroup");
  factory.registerNodeType<ForbidCollisionPairs>("ForbidCollisionPairs");
  factory.registerNodeType<CreateMTCGeneratePlacePose>("CreateMTCGeneratePlacePose");

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
