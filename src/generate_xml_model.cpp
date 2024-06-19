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
#include <behaviortree_mtc/create_mtc_joint_interpolation.h>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"

using namespace BT;
using namespace BT::MTC;

int main(int argc, char** argv)
{
  
  BehaviorTreeFactory factory;

  factory.registerNodeType<GeometryMsgsPose>("GeometryMsgsPose");
  factory.registerNodeType<GeometryMsgsTwistStamped>("GeometryMsgsTwistStamped");
  factory.registerNodeType<GeometryMsgsPoseStamped>("GeometryMsgsPoseStamped");
  factory.registerNodeType<GeometryMsgsVector3Stamped>("GeometryMsgsVector3Stamped");
  factory.registerNodeType<MoveItMsgsCollisionObjectCylinder>("MoveItMsgsCollisionObjectCylinder");
  factory.registerNodeType<MoveItMsgsCollisionObjectBox>("MoveItMsgsCollisionObjectBox");
  factory.registerNodeType<CreatePlanningSceneInterface>("CreatePlanningSceneInterface");
  factory.registerNodeType<AddObjectToPlanningScene>("AddObjectToPlanningScene");
  factory.registerNodeType<MoveMTCContainerToParentContainer>("MoveMTCContainerToParentContainer");
  factory.registerNodeType<MoveMTCStageToContainer>("MoveMTCStageToContainer");
  factory.registerNodeType<InitializeMTCTask>("InitializeMTCTask");
  factory.registerNodeType<CreateMTCCurrentState>("CreateMTCCurrentState");
  factory.registerNodeType<CreateMTCPipelinePlanner>("CreateMTCPipelinePlanner");
  factory.registerNodeType<CreateMTCCartesianPath>("CreateMTCCartesianPath");
  factory.registerNodeType<CreateMTCJointInterpolation>("CreateMTCJointInterpolation");
  factory.registerNodeType<PlanMTCTask>("PlanMTCTask");
  factory.registerNodeType<GetMTCRawStage>("GetMTCRawStage");
  factory.registerNodeType<CreateMTCMoveToNamedJointPose>("CreateMTCMoveToNamedJointPose");
  factory.registerNodeType<CreateMTCMoveToJoint>("CreateMTCMoveToJoint");
  factory.registerNodeType<CreateMTCMoveToPoint>("CreateMTCMoveToPoint");
  factory.registerNodeType<CreateMTCMoveToPose>("CreateMTCMoveToPose");
  factory.registerNodeType<CreateMTCMoveRelativeTranslate>("CreateMTCMoveRelativeTranslate");
  factory.registerNodeType<CreateMTCMoveRelativeJoint>("CreateMTCMoveRelativeJoint");
  factory.registerNodeType<CreateMTCMoveRelativeTwist>("CreateMTCMoveRelativeTwist");
  factory.registerNodeType<CreateMTCConnect>("CreateMTCConnect");
  factory.registerNodeType<CreateMTCSerialContainer>("CreateMTCSerialContainer");
  factory.registerNodeType<CreateMTCComputeIK>("CreateMTCComputeIK");
  factory.registerNodeType<ConfigureInitFromMTCProperties<moveit::task_constructor::Stage>>("ConfigureInitFromMTCProperties");
  factory.registerScriptingEnums<moveit::task_constructor::Stage::PropertyInitializerSource>();
  factory.registerNodeType<CreateMTCModifyPlanningSceneAttachObjects>("CreateMTCModifyPlanningSceneAttachObjects");
  factory.registerNodeType<CreateMTCModifyPlanningSceneDetachObjects>("CreateMTCModifyPlanningSceneDetachObjects");
  factory.registerNodeType<ForbidCollisionPairs>("ForbidCollisionPairs");
  factory.registerNodeType<AllowCollisionPairs>("AllowCollisionPairs");
  factory.registerNodeType<AllowAllCollisions>("AllowAllCollisions");
  factory.registerNodeType<ForbidAllCollisions>("ForbidAllCollisions");
  factory.registerNodeType<AllowCollisionsJointModelGroup>("AllowCollisionsJointModelGroup");
  factory.registerNodeType<ForbidCollisionsJointModelGroup>("ForbidCollisionsJointModelGroup");
  factory.registerNodeType<CreateMTCGenerateGraspPose>("CreateMTCGenerateGraspPose");
  factory.registerNodeType<CreateMTCGeneratePlacePose>("CreateMTCGeneratePlacePose");

  std::string xml_models = BT::writeTreeNodesModelXML(factory);
  std::string filePath = "./config/bt_mtc.xml";
  std::ofstream file(filePath);
    if (file.is_open())
    {
        file << xml_models;
        file.close();
        std::cout << "File saved successfully: " << filePath << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file: " << filePath << std::endl;
    }

  return 0;
}