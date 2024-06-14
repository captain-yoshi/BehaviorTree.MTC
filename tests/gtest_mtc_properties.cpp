#include <gtest/gtest.h>
#include <behaviortree_cpp/bt_factory.h>

#include <behaviortree_mtc/set_mtc_properties.h>
#include <behaviortree_mtc/expose_mtc_properties.h>

#include <behaviortree_mtc/std_containers.h>  // convertFromString -> property_names

#include <moveit/task_constructor/stages/current_state.h>

using BT::NodeStatus;

using bt_mtc::ExposeMTCProperties;
using bt_mtc::SetMTCProperties;

using moveit::task_constructor::ContainerBase;
using moveit::task_constructor::Stage;
using moveit::task_constructor::Task;

constexpr double EPS_DOUBLE = std::numeric_limits<double>::epsilon();

TEST(MTCProperties, SetMTCProperties)
{
  BT::BehaviorTreeFactory factory;

  // clang-format off

  const std::string xml_text = R"(
    <root BTCPP_format="4" >
       <BehaviorTree>
          <Sequence>
             <SetMTCPropertiesBool stage="{mtc_stage}" property_name="bool<false>" property="false" />
             <SetMTCPropertiesBool stage="{mtc_stage}" property_name="bool<true>" property="true" />

             <SetMTCPropertiesInt stage="{mtc_stage}" property_name="int<-66>" property="-66" />
             <SetMTCPropertiesInt stage="{mtc_stage}" property_name="int<0>" property="0" />
             <SetMTCPropertiesInt stage="{mtc_stage}" property_name="int<1211>" property="1211" />

             <SetMTCPropertiesDouble stage="{mtc_stage}" property_name="double<-11.765>" property="-11.765" />
             <SetMTCPropertiesDouble stage="{mtc_stage}" property_name="double<0>" property="0" />
             <SetMTCPropertiesDouble stage="{mtc_stage}" property_name="double<2>" property="2" />
             <SetMTCPropertiesDouble stage="{mtc_stage}" property_name="double<5e35>" property="5e35" />

             <SetMTCPropertiesString stage="{mtc_stage}" property_name="string<>" property="" />
             <SetMTCPropertiesString stage="{mtc_stage}" property_name="string<BehaviorTree.CPP>" property="BehaviorTree.CPP" />

             <SetMTCPropertiesPose stage="{mtc_stage}" property_name="geometry_msgs::Pose" property="{pose}" />
          </Sequence>
       </BehaviorTree>
    </root>)";

  // clang-format on

  factory.registerNodeType<SetMTCProperties<moveit::task_constructor::Stage, bool>>("SetMTCPropertiesBool");
  factory.registerNodeType<SetMTCProperties<moveit::task_constructor::Stage, int>>("SetMTCPropertiesInt");
  factory.registerNodeType<SetMTCProperties<moveit::task_constructor::Stage, double>>("SetMTCPropertiesDouble");
  factory.registerNodeType<SetMTCProperties<moveit::task_constructor::Stage, std::string>>("SetMTCPropertiesString");
  factory.registerNodeType<SetMTCProperties<moveit::task_constructor::Stage, geometry_msgs::Pose>>("SetMTCPropertiesPose");

  auto bb = BT::Blackboard::create();

  // set blackboard values
  moveit::task_constructor::StagePtr stage = std::make_shared<moveit::task_constructor::stages::CurrentState>();
  bb->set("mtc_stage", stage);

  auto pose = geometry_msgs::Pose();
  pose.position.x = 7;
  pose.position.y = -2;
  pose.position.z = 3;
  pose.orientation.x = -0.3812674;
  pose.orientation.y = -0.7607049;
  pose.orientation.z = 0.4696361;
  pose.orientation.w = -0.2353829;
  bb->set("pose", pose);

  auto tree = factory.createTreeFromText(xml_text, bb);

  for(int i = 0; i < 5; i++)
  {
    NodeStatus status = tree.tickWhileRunning();
    ASSERT_EQ(status, NodeStatus::SUCCESS);
  }

  ASSERT_EQ(stage->properties().get<bool>("bool<false>"), false);
  ASSERT_EQ(stage->properties().get<bool>("bool<true>"), true);

  ASSERT_EQ(stage->properties().get<int>("int<-66>"), -66);
  ASSERT_EQ(stage->properties().get<int>("int<0>"), 0);
  ASSERT_EQ(stage->properties().get<int>("int<1211>"), 1211);

  ASSERT_NEAR(stage->properties().get<double>("double<-11.765>"), -11.765, EPS_DOUBLE);
  ASSERT_NEAR(stage->properties().get<double>("double<0>"), 0.0, EPS_DOUBLE);
  ASSERT_NEAR(stage->properties().get<double>("double<2>"), 2.0, EPS_DOUBLE);
  ASSERT_NEAR(stage->properties().get<double>("double<5e35>"), 5e35, EPS_DOUBLE);

  ASSERT_STREQ(stage->properties().get<std::string>("string<>").c_str(), "");
  ASSERT_STREQ(stage->properties().get<std::string>("string<BehaviorTree.CPP>").c_str(), "BehaviorTree.CPP");

  const auto& p = stage->properties().get<geometry_msgs::Pose>("geometry_msgs::Pose");
  EXPECT_NEAR(p.position.x, pose.position.x, EPS_DOUBLE);
  EXPECT_NEAR(p.position.y, pose.position.y, EPS_DOUBLE);
  EXPECT_NEAR(p.position.z, pose.position.z, EPS_DOUBLE);
  EXPECT_NEAR(p.orientation.x, pose.orientation.x, EPS_DOUBLE);
  EXPECT_NEAR(p.orientation.y, pose.orientation.y, EPS_DOUBLE);
  EXPECT_NEAR(p.orientation.z, pose.orientation.z, EPS_DOUBLE);
  EXPECT_NEAR(p.orientation.w, pose.orientation.w, EPS_DOUBLE);
}

TEST(MTCProperties, ExposeMTCProperties)
{
  BT::BehaviorTreeFactory factory;

  // clang-format off

  const std::string xml_text = R"(
    <root BTCPP_format="4" >
       <BehaviorTree>
          <AsyncSequence>
             <ExposeMTCStagePropertiesToStage from="{stage1}" to="{stage2}" property_names="timeout,timer,allow_collisions" />
             <ExposeMTCStagePropertiesToStage from="{stage1}" to="{stage2}" property_names="object_id" />
             <ExposeMTCStagePropertiesToContainer from="{stage1}" to="{container1}" property_names="timer,allow_collisions,object_id" />
             <ExposeMTCContainerPropertiesToContainer from="{container1}" to="{container2}" property_names="timer,allow_collisions,object_id" />
             <!-- stage2 properties has been cleared at this point -->
             <ExposeMTCContainerPropertiesToStage from="{container1}" to="{stage2}" property_names="timer,allow_collisions,object_id" />
             <!-- stage1 properties has been cleared at this point -->
             <ExposeMTCTaskPropertiesToStage from="{task}" to="{stage1}" property_names="group,eef" />
             <!-- container1 properties has been cleared at this point -->
             <ExposeMTCTaskPropertiesToContainer from="{task}" to="{container1}" property_names="group,eef" />
          </AsyncSequence>
       </BehaviorTree>
    </root>)";

  // clang-format on

  // factory.registerNodeType<ReturnRunningOnce>("ReturnRunningOnce");
  factory.registerNodeType<ExposeMTCProperties<Stage, Stage>>("ExposeMTCStagePropertiesToStage");
  factory.registerNodeType<ExposeMTCProperties<Stage, ContainerBase>>("ExposeMTCStagePropertiesToContainer");
  factory.registerNodeType<ExposeMTCProperties<ContainerBase, Stage>>("ExposeMTCContainerPropertiesToStage");
  factory.registerNodeType<ExposeMTCProperties<ContainerBase, ContainerBase>>("ExposeMTCContainerPropertiesToContainer");
  factory.registerNodeType<ExposeMTCProperties<Task, Stage>>("ExposeMTCTaskPropertiesToStage");
  factory.registerNodeType<ExposeMTCProperties<Task, ContainerBase>>("ExposeMTCTaskPropertiesToContainer");

  auto bb = BT::Blackboard::create();

  // set blackboard values
  moveit::task_constructor::TaskPtr task = std::make_shared<Task>();
  moveit::task_constructor::ContainerBasePtr container1 = std::make_shared<moveit::task_constructor::SerialContainer>();
  moveit::task_constructor::ContainerBasePtr container2 = std::make_shared<moveit::task_constructor::Alternatives>();
  moveit::task_constructor::StagePtr stage1 = std::make_shared<moveit::task_constructor::stages::CurrentState>();
  moveit::task_constructor::StagePtr stage2 = std::make_shared<moveit::task_constructor::stages::CurrentState>();

  task->properties().set("group", "panda_arm");
  task->properties().set("eef", "hand");

  stage1->properties().set("timeout", 0.5);  // property already declared in CurrentState
  stage1->properties().set("timer", 0.5);
  stage1->properties().set("allow_collisions", true);
  stage1->properties().set("object_id", "cup");

  bb->set("task", task);
  bb->set("container1", container1);
  bb->set("container2", container2);
  bb->set("stage1", stage1);
  bb->set("stage2", stage2);

  auto tree = factory.createTreeFromText(xml_text, bb);

  // first tick: stage1 expose {timeout, timer, allow_collisions} to stage2
  double stage2_timeout = stage2->properties().get<double>("timeout");
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);

  ASSERT_EQ(stage2->properties().get<bool>("allow_collisions"), true);
  ASSERT_EQ(stage2->properties().get<double>("timeout"), stage2_timeout);  // a property already declared is not affected
  ASSERT_EQ(stage2->properties().get<double>("timer"), 0.5);
  EXPECT_THROW(stage2->properties().get<std::string>("object_id"), moveit::task_constructor::Property::undeclared);

  // second tick: stage1 expose {object_id} to stage2
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);

  ASSERT_EQ(stage2->properties().get<bool>("allow_collisions"), true);
  ASSERT_EQ(stage2->properties().get<double>("timeout"), stage2_timeout);
  ASSERT_EQ(stage2->properties().get<double>("timer"), 0.5);
  EXPECT_STREQ(stage2->properties().get<std::string>("object_id").c_str(), "cup");

  // third tick: stage1 expose {timer, allow_collisions, object_id} to container1
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);

  ASSERT_EQ(container1->properties().get<bool>("allow_collisions"), true);
  ASSERT_EQ(container1->properties().get<double>("timer"), 0.5);
  EXPECT_STREQ(container1->properties().get<std::string>("object_id").c_str(), "cup");

  // fourth tick: container1 expose {timer, allow_collisions, object_id} to container2
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);

  ASSERT_EQ(container2->properties().get<bool>("allow_collisions"), true);
  ASSERT_EQ(container2->properties().get<double>("timer"), 0.5);
  EXPECT_STREQ(container2->properties().get<std::string>("object_id").c_str(), "cup");

  // fifth tick: container1 expose {timer, allow_collisions, object_id} to stage2
  stage2->properties().reset();
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);

  ASSERT_EQ(stage2->properties().get<bool>("allow_collisions"), true);
  ASSERT_EQ(stage2->properties().get<double>("timer"), 0.5);
  EXPECT_STREQ(stage2->properties().get<std::string>("object_id").c_str(), "cup");

  // sixth tick: task expose {timer, allow_collisions, object_id} to stage1
  stage1->properties().reset();
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);

  EXPECT_STREQ(stage1->properties().get<std::string>("group").c_str(), "panda_arm");
  EXPECT_STREQ(stage1->properties().get<std::string>("eef").c_str(), "hand");

  // seventh tick: task expose {timer, allow_collisions, object_id} to container1
  container1->properties().reset();
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);

  EXPECT_STREQ(stage1->properties().get<std::string>("group").c_str(), "panda_arm");
  EXPECT_STREQ(stage1->properties().get<std::string>("eef").c_str(), "hand");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
