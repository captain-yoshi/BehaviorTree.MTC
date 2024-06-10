#include <gtest/gtest.h>
#include <behaviortree_cpp/bt_factory.h>

#include <behaviortree_mtc/set_mtc_properties.h>

#include <moveit/task_constructor/stages/current_state.h>

using BT::NodeStatus;
using namespace bt_mtc;

constexpr double EPS_DOUBLE = std::numeric_limits<double>::epsilon();

TEST(MTCProperties, SetMTCProperties)
{
  BT::BehaviorTreeFactory factory;

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

  factory.registerNodeType<SetMTCProperties<bool>>("SetMTCPropertiesBool");
  factory.registerNodeType<SetMTCProperties<int>>("SetMTCPropertiesInt");
  factory.registerNodeType<SetMTCProperties<double>>("SetMTCPropertiesDouble");
  factory.registerNodeType<SetMTCProperties<std::string>>("SetMTCPropertiesString");
  factory.registerNodeType<SetMTCProperties<geometry_msgs::Pose>>("SetMTCPropertiesPose");

  auto bb = BT::Blackboard::create();

  // set blackboard values
  moveit::task_constructor::StagePtr stage = std::make_shared<moveit::task_constructor::stages::CurrentState>();
  bb->set("mtc_stage", stage);

  auto pose = std::make_shared<geometry_msgs::Pose>();
  pose->position.x = 7;
  pose->position.y = -2;
  pose->position.z = 3;
  pose->orientation.x = -0.3812674;
  pose->orientation.y = -0.7607049;
  pose->orientation.z = 0.4696361;
  pose->orientation.w = -0.2353829;
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
  EXPECT_NEAR(p.position.x, pose->position.x, EPS_DOUBLE);
  EXPECT_NEAR(p.position.y, pose->position.y, EPS_DOUBLE);
  EXPECT_NEAR(p.position.z, pose->position.z, EPS_DOUBLE);
  EXPECT_NEAR(p.orientation.x, pose->orientation.x, EPS_DOUBLE);
  EXPECT_NEAR(p.orientation.y, pose->orientation.y, EPS_DOUBLE);
  EXPECT_NEAR(p.orientation.z, pose->orientation.z, EPS_DOUBLE);
  EXPECT_NEAR(p.orientation.w, pose->orientation.w, EPS_DOUBLE);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
