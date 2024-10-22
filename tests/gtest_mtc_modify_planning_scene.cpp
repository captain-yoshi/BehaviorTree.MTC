#include <gtest/gtest.h>
#include <behaviortree_cpp/bt_factory.h>

#include <behaviortree_mtc/create_mtc_modify_planning_scene.h>

#include <behaviortree_mtc/std_containers.h>  // convertFromString ->

#include <moveit/task_constructor/stages/modify_planning_scene.h>

using BT::NodeStatus;

using BT::MTC::CreateMTCModifyPlanningSceneAttachObjects;
using BT::MTC::CreateMTCModifyPlanningSceneDetachObjects;

using moveit::task_constructor::stages::ModifyPlanningScene;

TEST(ModifyPlanningScene, AttachObjects)
{
  BT::BehaviorTreeFactory factory;

  // clang-format off

  const std::string xml_text = R"(
    <root BTCPP_format="4" >
       <BehaviorTree>
          <AsyncSequence>
             <AttachObjects stage_name="attach cylinder1" object_names="cylinder1" link_name="panda_link7" stage="{stage_attach_cylinder1}" />
             <AttachObjects stage_name="attach cylinders" object_names="cylinder1,cylinder2" link_name="panda_link7" stage="{stage_attach_cylinders}" />
          </AsyncSequence>
       </BehaviorTree>
    </root>)";

  // clang-format on

  factory.registerNodeType<CreateMTCModifyPlanningSceneAttachObjects>("AttachObjects");

  auto bb = BT::Blackboard::create();

  auto tree = factory.createTreeFromText(xml_text, bb);

  // first tick: attach objects {cylinder1} to link panda_link7
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);

  moveit::task_constructor::StagePtr stage1{ nullptr };
  EXPECT_NO_THROW(stage1 = bb->get<moveit::task_constructor::StagePtr>("stage_attach_cylinder1"));
  EXPECT_TRUE(stage1 != nullptr);
  EXPECT_STREQ(stage1->name().c_str(), "attach cylinder1");

  // second tick: attach objects {cylinder1, cylinder2} to link panda_link7
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);

  moveit::task_constructor::StagePtr stage2{ nullptr };
  EXPECT_NO_THROW(stage2 = bb->get<moveit::task_constructor::StagePtr>("stage_attach_cylinders"));
  EXPECT_TRUE(stage2 != nullptr);
  EXPECT_STREQ(stage2->name().c_str(), "attach cylinders");
}

TEST(ModifyPlanningScene, DetachObjects)
{
  BT::BehaviorTreeFactory factory;

  // clang-format off

  const std::string xml_text = R"(
    <root BTCPP_format="4" >
       <BehaviorTree>
          <AsyncSequence>
             <DetachObjects stage_name="detach cylinder1" object_names="cylinder1" link_name="panda_link7" stage="{stage_detach_cylinder1}" />
             <DetachObjects stage_name="detach cylinders" object_names="cylinder1,cylinder2" link_name="panda_link7" stage="{stage_detach_cylinders}" />
          </AsyncSequence>
       </BehaviorTree>
    </root>)";

  // clang-format on

  factory.registerNodeType<CreateMTCModifyPlanningSceneDetachObjects>("DetachObjects");

  auto bb = BT::Blackboard::create();

  auto tree = factory.createTreeFromText(xml_text, bb);

  // first tick: detach objects {cylinder1} from link panda_link7
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);

  moveit::task_constructor::StagePtr stage1{ nullptr };
  EXPECT_NO_THROW(stage1 = bb->get<moveit::task_constructor::StagePtr>("stage_detach_cylinder1"));
  EXPECT_TRUE(stage1 != nullptr);
  EXPECT_STREQ(stage1->name().c_str(), "detach cylinder1");

  // second tick: detach objects {cylinder1, cylinder2} from link panda_link7
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::SUCCESS);

  moveit::task_constructor::StagePtr stage2{ nullptr };
  EXPECT_NO_THROW(stage2 = bb->get<moveit::task_constructor::StagePtr>("stage_detach_cylinders"));
  EXPECT_TRUE(stage2 != nullptr);
  EXPECT_STREQ(stage2->name().c_str(), "detach cylinders");
}
