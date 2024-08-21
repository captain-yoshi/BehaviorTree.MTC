#include <rclcpp/rclcpp.hpp>

#include <behaviortree_mtc/initialize_mtc_task.h>
#include <behaviortree_mtc/create_mtc_current_state.h>
#include <behaviortree_mtc/create_mtc_move_to.h>
#include <behaviortree_mtc/create_mtc_pipeline_planner.h>
#include <behaviortree_mtc/move_mtc_stage_to_container.h>
#include <behaviortree_mtc/plan_mtc_task.h>

#include <behaviortree_mtc/geometry_msgs.h>

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
       <InitializeMTCTask        task="{mtc_task}" container="{container}" />
       <CreateMTCPipelinePlanner pipeline_id="ompl"
                                 planner_id="RRTConnect"
                                 solver="{rrt_connect}"/>
       <CreateMTCCurrentState    stage="{stage}" />
       <MoveMTCStageToContainer  container="{container}" stage="{stage}" />

       <!-- joint named position motion -->
       <CreateMTCMoveToNamedJointPose name="move to -> close_hand motion"
                                      group="hand"
                                      solver="{rrt_connect}"
                                      goal="close"
                                      stage="{stage_move_to_close_hand}" />
       <MoveMTCStageToContainer  container="{container}" stage="{stage_move_to_close_hand}" />
       <CreateMTCMoveToNamedJointPose name="move to -> open_hand motion"
                                      group="hand"
                                      solver="{rrt_connect}"
                                      goal="open"
                                      stage="{stage_move_to_open_hand}" />
       <MoveMTCStageToContainer  container="{container}" stage="{stage_move_to_open_hand}" />
       
       <!-- joint motion -->
       <CreateMTCMoveToJoint       name="move to -> joint motion"
                                   group="panda_arm"
                                   solver="{rrt_connect}"
                                   goal="panda_joint7:-1.57079632679,panda_joint6:3.14"
                                   stage="{stage_move_to_joint}" />
       <MoveMTCStageToContainer  container="{container}" stage="{stage_move_to_joint}" />
       
       <!-- cartesian pose motion -->
       <GeometryMsgsPoseStamped frame_id="panda_link8" position="0,0,0" quaternion="1,0,0,0" pose_stamped="{ik_frame}"/>
       <GeometryMsgsPoseStamped frame_id="panda_link8" position="0,0,0.05" quaternion="0,0,0,0" pose_stamped="{goal_pose}"/>
       <CreateMTCMoveToPose     name="move to -> cartesian pose motion"
                                group="panda_arm"
                                solver="{rrt_connect}"
                                ik_frame="{ik_frame}"
                                goal="{goal_pose}"
                                stage="{stage_move_to_pose}" />
      <MoveMTCStageToContainer  container="{container}" stage="{stage_move_to_pose}" />
      
      <!-- cartesian point motion -->
       <GeometryMsgsPointStamped frame_id="panda_link8" point="0,0.05,0" point_stamped="{goal_point}"/>
       <CreateMTCMoveToPoint     name="move to -> cartesian point motion"
                                 group="panda_arm"
                                 solver="{rrt_connect}"
                                 ik_frame="{ik_frame}"
                                 goal="{goal_point}"
                                 stage="{stage_move_to_point}" />
      <MoveMTCStageToContainer  container="{container}" stage="{stage_move_to_point}" />
       <PlanMTCTask              task="{mtc_task}" max_solutions="5" />
     </Sequence>
   </BehaviorTree>

 </root>
 )";
// clang-format on

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  
  // Declare parameters passed by a launch file
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("test_behavior_tree", options);

  BehaviorTreeFactory factory;

  factory.registerNodeType<InitializeMTCTask>("InitializeMTCTask", BT::RosNodeParams(node));
  factory.registerNodeType<CreateMTCPipelinePlanner>("CreateMTCPipelinePlanner", BT::RosNodeParams(node));
  factory.registerNodeType<CreateMTCCurrentState>("CreateMTCCurrentState");
  factory.registerNodeType<CreateMTCMoveToNamedJointPose>("CreateMTCMoveToNamedJointPose");
  factory.registerNodeType<CreateMTCMoveToJoint>("CreateMTCMoveToJoint");
  factory.registerNodeType<CreateMTCMoveToPose>("CreateMTCMoveToPose");
  factory.registerNodeType<CreateMTCMoveToPoint>("CreateMTCMoveToPoint");
  factory.registerNodeType<MoveMTCStageToContainer>("MoveMTCStageToContainer");
  factory.registerNodeType<PlanMTCTask>("PlanMTCTask");

  factory.registerNodeType<GeometryMsgsPoseStamped>("GeometryMsgsPoseStamped");
  factory.registerNodeType<GeometryMsgsPointStamped>("GeometryMsgsPointStamped");

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
