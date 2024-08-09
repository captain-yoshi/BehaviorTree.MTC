#include <rclcpp/rclcpp.hpp>

#include <behaviortree_mtc/initialize_mtc_task.h>
#include <behaviortree_mtc/create_mtc_current_state.h>
#include <behaviortree_mtc/move_mtc_stage_to_container.h>
#include <behaviortree_mtc/plan_mtc_task.h>
#include <behaviortree_mtc/moveit_msgs.h>
#include <behaviortree_mtc/geometry_msgs.h>
#include <behaviortree_mtc/create_mtc_pipeline_planner.h>
#include <behaviortree_mtc/create_planning_scene_interface.h>
#include <behaviortree_mtc/create_mtc_modify_planning_scene.h>
#include <behaviortree_mtc/add_object_to_planning_scene.h>
#include <behaviortree_mtc/plan_mtc_task.h>

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
            <AddObjectToPlanningScene  planning_scene_interface="{psi}"
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
            <InitializeMTCTask        task="{mtc_task}" container="{task_container}" />
            <CreateMTCPipelinePlanner pipeline_id="ompl"
                                 planner_id="RRTConnect"
                                 solver="{rrt_connect}"
                                 goal_joint_tolerance="1e-5" />
            <CreateMTCCurrentState    stage="{current_state}" />
            <MoveMTCStageToContainer  container="{task_container}" stage="{current_state}" />
            <AllowCollisionsJointModelGroup stage_name="allow collisions -> hand,object"
                                            stage="{allow_collisions}"
                                            objects="{object_name}"
                                            jmg_name="{hand_group_name}"
                                            task="{mtc_task}" />
            <MoveMTCStageToContainer  container="{task_container}"  stage="{allow_collisions}" />
            <ForbidAllCollisions  stage_name="forbid collisions -> hand,object"
                                  stage="{forbid_collisions}"
                                  objects="{object_name}" />
            <MoveMTCStageToContainer  container="{task_container}"  stage="{forbid_collisions}" />
            <PlanMTCTask              task="{mtc_task}" max_solutions="5" />
        </Sequence>
     </BehaviorTree>

 </root>
 )";

// clang-format on

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_behavior_tree");

  BehaviorTreeFactory factory;

  factory.registerNodeType<GeometryMsgsPose>("GeometryMsgsPose");
  factory.registerNodeType<MoveItMsgsCollisionObjectCylinder>("MoveItMsgsCollisionObjectCylinder");
  factory.registerNodeType<MoveItMsgsCollisionObjectBox>("MoveItMsgsCollisionObjectBox");
  factory.registerNodeType<CreatePlanningSceneInterface>("CreatePlanningSceneInterface");
  factory.registerNodeType<AddObjectToPlanningScene>("AddObjectToPlanningScene");
  factory.registerNodeType<MoveMTCStageToContainer>("MoveMTCStageToContainer");
  factory.registerNodeType<InitializeMTCTask>("InitializeMTCTask");
  factory.registerNodeType<CreateMTCCurrentState>("CreateMTCCurrentState");
  factory.registerNodeType<CreateMTCPipelinePlanner>("CreateMTCPipelinePlanner");
  factory.registerNodeType<PlanMTCTask>("PlanMTCTask");
  factory.registerNodeType<AllowCollisionPairs>("AllowCollisionPairs");
  factory.registerNodeType<ForbidAllCollisions>("ForbidAllCollisions");
  factory.registerNodeType<AllowCollisionsJointModelGroup>("AllowCollisionsJointModelGroup");

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

  // Lambda function for the timer callback to tick the behavior tree
  auto tick_tree = [&tree, &status]() {
    if (status == NodeStatus::IDLE || status == NodeStatus::RUNNING)
    {
      status = tree.tickExactlyOnce();
      std::cout << "Status = " << status << std::endl;
    }
  };

  // Timer to tick the behavior tree
  auto timer = node->create_wall_timer(std::chrono::milliseconds(10), tick_tree);

  // Spin the node (this will process callbacks including the timer)
  rclcpp::spin(node);

  // Shutdown when user press CTRL-C
  rclcpp::shutdown();

  return 0;
}
