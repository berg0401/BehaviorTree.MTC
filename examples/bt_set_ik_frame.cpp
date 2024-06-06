#include <ros/ros.h>

#include <behaviortree_mtc/initialize_mtc_task.h>
#include <behaviortree_mtc/create_mtc_current_state.h>
#include <behaviortree_mtc/move_mtc_stage_to_container.h>
#include <behaviortree_mtc/plan_mtc_task.h>
#include <behaviortree_mtc/moveit_msgs.h>
#include <behaviortree_mtc/geometry_msgs.h>
#include <behaviortree_mtc/create_mtc_pipeline_planner.h>
#include <behaviortree_mtc/create_planning_scene_interface.h>
#include <behaviortree_mtc/set_up_scene.h>
#include <behaviortree_mtc/plan_mtc_task.h>
#include <behaviortree_mtc/create_mtc_generate_grasp_pose.h>
#include <behaviortree_mtc/get_mtc_raw_stage.h>
#include <behaviortree_mtc/create_mtc_move_to.h>
#include <behaviortree_mtc/create_mtc_connect.h>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

using namespace BT;
using namespace bt_mtc;

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
            <CreatePlanningSceneInterface  planning_scene_interface="{psi}" />
            <GeometryMsgsPose  name="box's pose"
                               position="0.5,-0.25,-0.05"
                               quaternion="1,0,0,0"
                               pose="{box_pose}" />
            <MoveItMsgsCollisionObjectBox object_name="{table_name}" 
                                               object_dimensions="0.4,0.5,0.1"
                                               pose="{box_pose}"
                                               object_reference_frame="{reference_frame}"
                                               collision_object="{box}"  />
            <SetUpScene   planning_scene_interface="{psi}"
                          collision_object="{box}"  />
            <GeometryMsgsPose  name="cylinder's pose"
                               position="0.5,-0.25,0.125"
                               quaternion="1,0,0,0"
                               pose="{cylinder_pose}" />
            <MoveItMsgsCollisionObjectCylinder  object_name="{object_name}" 
                                                object_dimensions="0.25,0.02"
                                                pose="{cylinder_pose}"
                                                object_reference_frame="world"
                                                collision_object="{cylinder}"  />
            <SetUpScene  planning_scene_interface="{psi}" 
                         collision_object="{cylinder}"/> 
            <InitializeMTCTask        task="{mtc_task}" container="{container}" />
            <CreateMTCPipelinePlanner pipeline_id="ompl"
                                 planner_id="RRTConnect"
                                 solver="{rrt_connect}" />
            <CreateMTCCurrentState    stage="{stage}" />
            <MoveMTCStageToContainer  container="{container}" stage="{stage}" />
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
            <GetMTCRawStage  stage="{stage_move_to_open_hand}" raw_stage="{raw_stage}"  />
            <MoveMTCStageToContainer  container="{container}" stage="{stage_move_to_open_hand}" />
            <CreateMTCConnect  stage_name="MoveToPick"
                               timeout="5.0"
                               group="panda_arm"
                               planner="{rrt_connect}"
                               stage="{connect}" />
            <MoveMTCStageToContainer  container="{container}" stage="{connect}" />  
            <CreateMTCGenerateGraspPose  stage_name="generate_grasp_pose"
                                         eef="{eef}"
                                         object="{object_name}"
                                         angle_delta="{angle}"
                                         pregrasp_pose="open"
                                         monitored_stage="{raw_stage}"
                                         stage="{generate_pose}" /> 
            <MoveMTCStageToContainer stage="{generate_pose}" container="{container}" /> 
            <PlanMTCTask              task="{mtc_task}" max_solutions="5" />
        </Sequence>
     </BehaviorTree>

 </root>
 )";
// clang-format on

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_behavior_tree");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  BehaviorTreeFactory factory;

  factory.registerNodeType<GeometryMsgsPose>("GeometryMsgsPose");
  factory.registerNodeType<MoveItMsgsCollisionObjectCylinder>("MoveItMsgsCollisionObjectCylinder");
  factory.registerNodeType<MoveItMsgsCollisionObjectBox>("MoveItMsgsCollisionObjectBox");
  factory.registerNodeType<CreatePlanningSceneInterface>("CreatePlanningSceneInterface");
  factory.registerNodeType<SetUpScene>("SetUpScene");
  factory.registerNodeType<CreateMTCGenerateGraspPose>("CreateMTCGenerateGraspPose");
  factory.registerNodeType<MoveMTCStageToContainer>("MoveMTCStageToContainer");
  factory.registerNodeType<InitializeMTCTask>("InitializeMTCTask");
  factory.registerNodeType<CreateMTCCurrentState>("CreateMTCCurrentState");
  factory.registerNodeType<CreateMTCPipelinePlanner>("CreateMTCPipelinePlanner");
  factory.registerNodeType<PlanMTCTask>("PlanMTCTask");
  factory.registerNodeType<GetMTCRawStage>("GetMTCRawStage");
  factory.registerNodeType<CreateMTCMoveToNamedJointPose>("CreateMTCMoveToNamedJointPose");
  factory.registerNodeType<CreateMTCConnect>("CreateMTCConnect");

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






