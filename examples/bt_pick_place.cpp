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
            <PlanningSceneInterface  planning_scene_interface="{psi}" />

            <!-- Create Table -->
            <PoseMsg name="box's pose" position="0.5,-0.25,-0.05" quaternion="1,0,0,0" pose="{box_pose}" />
            <CollisionObjectBoxMsg object_id="{table_name}" length="0.4" width="0.5" height="0.1" pose="{box_pose}" frame_id="{reference_frame}" collision_object="{box}" />
            <AddCollisionObjectToPlanningSceneInterface planning_scene_interface="{psi}" collision_object="{box}" />

            <!-- Create Cylinder -->
            <PoseMsg name="cylinder's pose" position="0.5,-0.25,0.125" quaternion="1,0,0,0" pose="{cylinder_pose}" />
            <CollisionObjectCylinderMsg object_id="{object_name}"
                                        radius="0.02"
                                        height="0.25"
                                        pose="{cylinder_pose}"
                                        frame_id="{reference_frame}"
                                        collision_object="{cylinder}" />
            <AddCollisionObjectToPlanningSceneInterface planning_scene_interface="{psi}" collision_object="{cylinder}" />

            <!-- Create Task -->
            <Task task="{mtc_task}" />

            <!-- Add properpties to the Task -->
            <SetTaskPropertiesString  stage="{mtc_task}"  property_name="eef"  property="{eef}" />
            <SetTaskPropertiesString  stage="{mtc_task}"  property_name="group"  property="{arm_group_name}" />
            <SetTaskPropertiesString  stage="{mtc_task}"  property_name="hand"  property="{hand_group_name}" />
            <SetTaskPropertiesString  stage="{mtc_task}"  property_name="hand_grasping_frame"  property="{hand_frame}" />
            <SetTaskPropertiesString  stage="{mtc_task}"  property_name="ik_frame"  property="{hand_frame}" />

            <!-- Create Solvers -->
            <PipelinePlanner pipeline_id="ompl" planner_id="RRTConnect" solver="{rrt_connect}" goal_joint_tolerance="1e-5" />
            <CartesianPath solver="{cartesian_planner}" max_velocity_scaling_factor="1.0" max_acceleration_scaling_factor="1.0" />

            <!-- Current State -->
            <CurrentState stage="{current_state}" />
            <MoveStageToTask child="{current_state}" parent="{mtc_task}" />


            <!-- Open Hand -->
            <PoseStampedMsg  frame_id="{hand_frame}" position="0,0,0" quaternion="1,0,0,0" pose_stamped="{ik_frame}" />
            <MoveToNamedJointPose name="move to -> open_hand motion"
                                        group="{hand_group_name}"
                                        solver="{rrt_connect}"
                                        goal="open"
                                        stage="{stage_move_to_open_hand}" />
            <GetStageRawPtr stage="{stage_move_to_open_hand}" raw_stage="{raw_stage}" />
            <MoveStageToTask child="{stage_move_to_open_hand}" parent="{mtc_task}" />


            <!-- Move to Pick -->
            <Connect stage_name="MoveToPick"
                     timeout="5.0"
                     group="{arm_group_name}"
                     planner="{rrt_connect}"
                     stage="{connect}" />
            <ConfigureStagePropertiesInitFrom stage="{connect}" property_names="" source_flag="PARENT" />
            <MoveStageToTask child="{connect}" parent="{mtc_task}" />

            <!-- Pick Object-->
            <SerialContainer container_name="pick object" container="{pick_object}" />
            <ExposeTaskPropertiesToContainer from="{mtc_task}" to="{pick_object}" property_names="group,eef,hand,ik_frame" />
            <ConfigureContainerPropertiesInitFrom stage="{pick_object}" property_names="group,eef,hand,ik_frame" source_flag="PARENT" />

                <!-- Approach Object -->
                <Vector3StampedMsg frame_id="{hand_frame}" vector="0,0,1" vector3_stamped="{tcp_translate}"/>
                <MoveRelativeTranslate stage_name="approach object"
                                       stage="{approach_object}"
                                       solver="{cartesian_planner}"
                                       ik_frame="{ik_frame}"
                                       direction="{tcp_translate}"
                                       min_distance="0.1"
                                       max_distance="0.15" />
                <ConfigureStagePropertiesInitFrom stage="{approach_object}" property_names="group" source_flag="PARENT" />
                <MoveStageToContainer child="{approach_object}" parent="{pick_object}" />

                <!-- Generate Grasp Pose -->
                <GenerateGraspPose stage_name="generate_grasp_pose"
                                   object="{object_name}"
                                   angle_delta="{angle}"
                                   pregrasp_pose="open"
                                   monitored_stage="{raw_stage}"
                                   stage="{generate_pose}" />
                <ConfigureStagePropertiesInitFrom stage="{generate_pose}" property_names="" source_flag="PARENT" />
                <PoseStampedMsg frame_id="{hand_frame}" position="0,0,0.1" quaternion="0.270595,0.653228,-0.270861,0.653228" pose_stamped="{ik_frame2}"/>
                <ComputeIK stage_name="grasp pose IK"
                           wrapped_stage="{generate_pose}"
                           max_ik_solutions="8"
                           min_solution_distance="1.0"
                           stage="{grasp_pose_IK}"
                           ik_frame="{ik_frame2}"  />
                <ConfigureStagePropertiesInitFrom  stage="{grasp_pose_IK}" property_names="eef,group" source_flag="PARENT" />
                <ConfigureStagePropertiesInitFrom  stage="{grasp_pose_IK}" property_names="target_pose" source_flag="INTERFACE" />
                <MoveStageToContainer child="{grasp_pose_IK}" parent="{pick_object}" />

                <!-- Allow Collisions (hand object) -->
                <ModifyPlanningSceneAllowCollisionsJointModelGroup stage_name="allow collisions -> hand,object"
                                                                   stage="{allow_collisions_object_robot}"
                                                                   objects="{object_name}"
                                                                   jmg_name="{hand_group_name}"
                                                                   task="{mtc_task}" />
                <MoveStageToContainer child="{allow_collisions_object_robot}" parent="{pick_object}" />

                <!-- Close Hand -->
                <MoveToNamedJointPose name="close hand"
                                      group="{hand_group_name}"
                                      solver="{rrt_connect}"
                                      goal="close"
                                      stage="{stage_move_to_close_hand}" />
                <MoveStageToContainer child="{stage_move_to_close_hand}" parent="{pick_object}" />

                <!-- Attach Object -->
                <ModifyPlanningSceneAttachObjects stage_name="attach object"
                                                  stage="{attach_object}"
                                                  object_names="{object_name}"
                                                  link_name="{hand_frame}" />
                <MoveStageToContainer child="{attach_object}" parent="{pick_object}" />

                <!-- Allow Collision (object support) -->
                <ModifyPlanningSceneAllowCollisionPairs stage_name="allow collisions -> object,support"
                                                        stage="{allow_collisions_table}"
                                                        first="{object_name}"
                                                        second="table" />
                <MoveStageToContainer child="{allow_collisions_table}" parent="{pick_object}" />

                <!-- Lift Object -->
                <Vector3StampedMsg frame_id="{reference_frame}" vector="0,0,1" vector3_stamped="{tcp_translate}"/>
                <MoveRelativeTranslate stage_name="lift object"
                                       stage="{lift_object}"
                                       solver="{cartesian_planner}"
                                       ik_frame="{ik_frame}"
                                       direction="{tcp_translate}"
                                       min_distance="0.01"
                                       max_distance="0.1" />
                <ConfigureStagePropertiesInitFrom stage="{lift_object}" property_names="group" source_flag="PARENT" />
                <MoveStageToContainer child="{lift_object}" parent="{pick_object}" />

                <!-- Forbid Collision (object support) -->
                <ModifyPlanningSceneForbidCollisionPairs stage_name="forbid collisions -> object,support"
                                                         stage="{forbid_collisions_table}"
                                                         first="{object_name}"
                                                         second="table" />
                <GetStageRawPtr stage="{forbid_collisions_table}" raw_stage="{raw_forbid_collisions_table}"  />
                <MoveStageToContainer child="{forbid_collisions_table}" parent="{pick_object}" />

                <!-- Add 'pick_object' Container to Task -->
                <MoveContainerToTask child="{pick_object}" parent="{mtc_task}" />

            <!-- Move to Place -->
            <Connect stage_name="Move To Place"
                     timeout="5.0"
                     group="{arm_group_name}"
                     planner="{rrt_connect}"
                     stage="{connectPlace}" />
            <ConfigureStagePropertiesInitFrom stage="{connectPlace}" property_names="" source_flag="PARENT" />
            <MoveStageToTask child="{connectPlace}" parent="{mtc_task}" />

            <!-- Place Object -->
            <SerialContainer container_name="place object" container="{place_object}" />
            <ExposeTaskPropertiesToContainer from="{mtc_task}" to="{place_object}" property_names="group,eef,hand" />
            <ConfigureContainerPropertiesInitFrom stage="{place_object}" property_names="group,eef,hand" source_flag="PARENT" />


                <!-- Lower Object -->
                <Vector3StampedMsg frame_id="{reference_frame}" vector="0,0,-1" vector3_stamped="{tcp_translate2}" />
                <MoveRelativeTranslate stage_name="lower object"
                                       stage="{lower_object}"
                                       solver="{cartesian_planner}"
                                       ik_frame="{ik_frame}"
                                       direction="{tcp_translate2}"
                                       min_distance="0.03"
                                       max_distance="0.13" />
                <ConfigureStagePropertiesInitFrom stage="{lower_object}" property_names="group" source_flag="PARENT" />
                <MoveStageToContainer child="{lower_object}" parent="{place_object}" />

                <!-- Generate Place Pose -->
                <PoseStampedMsg frame_id="world" position="0.6,-0.15,0.1251" quaternion="1,0,0,0" pose_stamped="{place_pose}" />
                <GeneratePlacePose stage="{generate_place_pose}"
                                   stage_name="generate place pose"
                                   object="{object_name}"
                                   pose="{place_pose}"
                                   monitored_stage="{raw_forbid_collisions_table}" />
                <ComputeIK stage_name="place pose IK"
                           wrapped_stage="{generate_place_pose}"
                           max_ik_solutions="2"
                           stage="{place_pose_IK}"
                           ik_frame="{ik_frame2}"  />
                <ConfigureStagePropertiesInitFrom  stage="{place_pose_IK}" property_names="eef,group" source_flag="PARENT" />
                <ConfigureStagePropertiesInitFrom  stage="{place_pose_IK}" property_names="target_pose" source_flag="INTERFACE" />
                <MoveStageToContainer child="{place_pose_IK}" parent="{place_object}" />

                <!-- Open Hand -->
                <MoveToNamedJointPose name="move to -> open_hand motion"
                                      group="{hand_group_name}"
                                      solver="{rrt_connect}"
                                      goal="open"
                                      stage="{stage_move_to_open_hand2}" />
                <MoveStageToContainer child="{stage_move_to_open_hand2}" parent="{place_object}" />

                <!-- Forbid Collision (hand, object) -->
                <ModifyPlanningSceneForbidCollisionsJointModelGroup stage_name="forbid collisions -> hand,object"
                                                                    stage="{forbid_collisions_object_robot}"
                                                                    objects="{object_name}"
                                                                    jmg_name="{hand_group_name}"
                                                                    task="{mtc_task}" />
                <MoveStageToContainer child="{forbid_collisions_object_robot}" parent="{place_object}" />

                <!-- Detach Object -->
                <ModifyPlanningSceneDetachObjects stage_name="detach object"
                                                  stage="{detach_object}"
                                                  object_names="{object_name}"
                                                  link_name="{hand_frame}" />
                <MoveStageToContainer child="{detach_object}" parent="{place_object}" />

                <!-- Retreat Motion -->
                <Vector3StampedMsg frame_id="{hand_frame}" vector="0,0,-1" vector3_stamped="{tcp_translate3}" />
                <MoveRelativeTranslate stage_name="retreat after place"
                                       stage="{retreat_after_place}"
                                       solver="{cartesian_planner}"
                                       ik_frame="{ik_frame}"
                                       direction="{tcp_translate3}"
                                       min_distance="0.12"
                                       max_distance="0.25" />
                <ConfigureStagePropertiesInitFrom stage="{retreat_after_place}" property_names="group" source_flag="PARENT" />
                <MoveStageToContainer child="{retreat_after_place}" parent="{place_object}" />

                <!-- Add 'place_object' Container to Task -->
                <MoveContainerToTask child="{place_object}" parent="{mtc_task}" />

            <!-- Move to Home -->
            <MoveToNamedJointPose name="move home"
                                  solver="{rrt_connect}"
                                  goal="ready"
                                  stage="{stage_move_to_home}" />
            <ConfigureStagePropertiesInitFrom stage="{stage_move_to_home}" property_names="group" source_flag="PARENT" />
            <MoveStageToTask child="{stage_move_to_home}" parent="{mtc_task}" />

            <!-- Plan the Task -->
            <PlanTask task="{mtc_task}" max_solutions="10" />
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
