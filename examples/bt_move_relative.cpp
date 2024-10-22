#include <rclcpp/rclcpp.hpp>

#include <behaviortree_mtc/initialize_mtc_task.h>
#include <behaviortree_mtc/create_mtc_current_state.h>
#include <behaviortree_mtc/create_mtc_move_relative.h>
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
                                 solver="{rrt_connect}" />
       <CreateMTCCurrentState    stage="{stage}" />
       <MoveMTCStageToContainer  container="{container}" stage="{stage}" />

       <!-- Translate Motion -> 200mm towards x axis wrt. panda_link8 -->
       <GeometryMsgsPoseStamped  frame_id="panda_link8" position="0,0,0" quaternion="1,0,0,0" pose_stamped="{ik_frame}"/>
       <GeometryMsgsVector3Stamped frame_id="panda_link8" vector="0.2,0,0" vector3_stamped="{tcp_translate}"/>
       <CreateMTCMoveRelativeTranslate name="move relative -> tcp translation"
                                       group="panda_arm"
                                       solver="{rrt_connect}"
                                       ik_frame="{ik_frame}"
                                       direction="{tcp_translate}"
                                       stage="{stage_move_rel_translate}" />
       <MoveMTCStageToContainer  container="{container}" stage="{stage_move_rel_translate}" />

       <!-- Twist Motion -->
       <GeometryMsgsPoseStamped  frame_id="panda_link8" position="0,0,0" quaternion="1,0,0,0" pose_stamped="{ik_frame}"/>
       <GeometryMsgsTwistStamped frame_id="panda_link8" linear_velocity="0,0,0" angular_velocity="0,0,1.57079632679" twist_stamped="{tcp_twist}"/>
       <CreateMTCMoveRelativeTwist name="move relative -> twist motion"
                                   group="panda_arm"
                                   solver="{rrt_connect}"
                                   ik_frame="{ik_frame}"
                                   direction="{tcp_twist}"
                                   stage="{stage_move_rel_twist}" />
       <MoveMTCStageToContainer  container="{container}" stage="{stage_move_rel_twist}" />

       <!-- Joint Motion -> return end joint to default value -->
       <GeometryMsgsPoseStamped  frame_id="panda_link8" position="0,0,0" quaternion="1,0,0,0" pose_stamped="{ik_frame}"/>
       <CreateMTCMoveRelativeJoint name="move relative -> joint motion"
                                   group="panda_arm"
                                   solver="{rrt_connect}"
                                   ik_frame="{ik_frame}"
                                   direction="panda_joint7:-1.57079632679"
                                   stage="{stage_move_rel_joint}" />
       <MoveMTCStageToContainer  container="{container}" stage="{stage_move_rel_joint}" />


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
  factory.registerNodeType<CreateMTCMoveRelativeTranslate>("CreateMTCMoveRelativeTranslate");
  factory.registerNodeType<CreateMTCMoveRelativeTwist>("CreateMTCMoveRelativeTwist");
  factory.registerNodeType<CreateMTCMoveRelativeJoint>("CreateMTCMoveRelativeJoint");
  factory.registerNodeType<MoveMTCStageToContainer>("MoveMTCStageToContainer");
  factory.registerNodeType<PlanMTCTask>("PlanMTCTask");

  factory.registerNodeType<GeometryMsgsPoseStamped>("GeometryMsgsPoseStamped");
  factory.registerNodeType<GeometryMsgsVector3Stamped>("GeometryMsgsVector3Stamped");
  factory.registerNodeType<GeometryMsgsTwistStamped>("GeometryMsgsTwistStamped");

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
