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
#include "behaviortree_cpp/xml_parsing.h"

using namespace BT;
using namespace BT::MTC;

int main(int argc, char** argv)
{
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

  std::string xml_models = BT::writeTreeNodesModelXML(factory);

  std::string path = PROJECT_WS;
  path += "/config/bt_mtc_model.xml";

  std::ofstream file(path);
  if(file.is_open())
  {
    file << xml_models;
    file.close();
    std::cout << "File saved successfully: " << path << std::endl;
  }
  else
  {
    std::cerr << "Unable to open file: " << path << std::endl;
  }

  return 0;
}
