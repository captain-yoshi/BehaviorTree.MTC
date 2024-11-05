#include <gtest/gtest.h>
#include <ros/ros.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_mtc/plan_mtc_task.h>
#include "stage_mockups.h"
#include "models.h"

using namespace BT;
using namespace BT::MTC;

TEST(PlanMTCTask, Preempt)
{
  BT::BehaviorTreeFactory factory;

  // clang-format off

  static const char* xml_text = R"(

   <root BTCPP_format="4" >
       <BehaviorTree ID="MainTree">
           <Sequence>
               <Script code="force_failure := false"/>

               <ReactiveSequence name="root">

                   <AlwaysSuccess _failureIf="force_failure==true" _onSuccess="force_failure=true"/>
                   <PlanMTCTask task="{mtc_task}" max_solutions="1"/>
         </ReactiveSequence>

       </Sequence>

     </BehaviorTree>
   </root>)";

  // clang-format off

  factory.registerNodeType<PlanMTCTask>("PlanMTCTask");


  // setup mtc task to be preempted
  moveit::task_constructor::TaskPtr task = std::make_shared<moveit::task_constructor::Task>();
  task->setRobotModel(getModel());

  auto timeout = std::chrono::milliseconds(10);
  task->add(std::make_unique<moveit::task_constructor::GeneratorMockup>(moveit::task_constructor::PredefinedCosts::constant(0.0)));
  task->add(std::make_unique<moveit::task_constructor::TimedForwardMockup>(timeout));

  // setup blackbaord
  auto bb = BT::Blackboard::create();
  bb->set("mtc_task", task);


  // test
  auto tree = factory.createTreeFromText(xml_text, bb);
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::FAILURE);
}
