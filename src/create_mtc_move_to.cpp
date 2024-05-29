#include <behaviortree_mtc/create_mtc_move_to.h>
#include <behaviortree_mtc/shared_to_unique.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;
using DirectionVariant = std::variant<std::shared_ptr<geometry_msgs::Vector3Stamped>,
                                      std::shared_ptr<std::map<std::string, double>>,
                                      std::shared_ptr<geometry_msgs::TwistStamped>>;

namespace
{
constexpr auto kPortStageName = "stage_name";
constexpr auto kPortStage = "stage";
constexpr auto kPortSolver = "solver";
constexpr auto kPortGroup = "group";
constexpr auto kPortTimeout = "timeout";
constexpr auto kPortIKFrame = "ik_frame";
constexpr auto kPortGoal = "goal";

}  // namespace

CreateMTCMoveToBase::CreateMTCMoveToBase(const std::string& name,
                                         const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreateMTCMoveToBase::tick(std::shared_ptr<moveit::task_constructor::stages::MoveTo>& stage)
{
  // Retrieve inputs
  std::string name;
  std::string group;
  MTC::solvers::PlannerInterfacePtr solver;
  std::shared_ptr<geometry_msgs::PoseStamped> ik_frame;
  double min_distance;
  double max_distance;
  double timeout;

  if(!getInput(kPortStageName, name) ||
     !getInput(kPortGroup, group) ||
     !getInput(kPortSolver, solver) ||
     !getInput(kPortIKFrame, ik_frame) ||
     !getInput(kPortTimeout, timeout))
    return NodeStatus::FAILURE;

  // Build stage
  stage = {
    new MTC::stages::MoveTo(name, solver),
    dirty::fake_deleter{}
  };

  stage->setGroup(group);
  stage->setIKFrame(*ik_frame);
  stage->setTimeout(timeout);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveToBase::providedPorts()
{
  return {
    BT::OutputPort<MTC::StagePtr>(kPortStage,"MoveTo stage"),
    BT::InputPort<std::string>(kPortStageName, "move to", "stage name"),
    BT::InputPort<std::string>(kPortGroup, "name of planning group"),
    BT::InputPort<double>(kPortTimeout, 1.0, ""),
    BT::InputPort<MTC::solvers::PlannerInterfacePtr>(kPortSolver, "planner interface"),
    BT::InputPort<std::shared_ptr<geometry_msgs::PoseStamped>>(kPortIKFrame, "frame to be moved in Cartesian direction"),
  };
}

CreateMTCMoveToNamedJointPose::CreateMTCMoveToNamedJointPose(const std::string& name,
                                                             const BT::NodeConfig& config)
  : CreateMTCMoveToBase(name, config)
{}

BT::NodeStatus CreateMTCMoveToNamedJointPose::tick()
{
  std::string named_joint_pose;
  //Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveTo> stage{ nullptr };
  auto node_status = CreateMTCMoveToBase::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;
  //Set goal
  if(!getInput(kPortGoal, named_joint_pose))
    return NodeStatus::FAILURE;
  stage->setGoal(*named_joint_pose);
  // Upcast to base class
  MTC::StagePtr base_stage = stage;
  setOutput(kPortStage, base_stage);
  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveToNamedJointPose::providedPorts()
{
  auto port_lists = CreateMTCMoveToBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::string>(kPortGoal, "name of the pose"));

  return port_lists;
}

CreateMTCMoveToJoint::CreateMTCMoveToJoint(const std::string& name,
                                           const BT::NodeConfig& config)
  : CreateMTCMoveToBase(name, config)
{}
BT::NodeStatus CreateMTCMoveToJoint::tick()
{
  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveTo> stage{ nullptr };
  auto node_status = CreateMTCMoveToBase::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  // Set goal
  std::shared_ptr<std::map<std::string, double>> joints{ nullptr };
  if(!getInput(kPortGoal, joints))
    return NodeStatus::FAILURE;

  stage->setGoal(*joints);

  // Upcast to base class
  MTC::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveToJoint::providedPorts()
{
  auto port_lists = CreateMTCMoveToBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<std::map<std::string, double>>>(kPortGoal, "move specified joint variables by given amount"));

  return port_lists;
}

CreateMTCMoveToPose::CreateMTCMoveToPose(const std::string& name,
                                         const BT::NodeConfig& config)
  : CreateMTCMoveToBase(name, config)
{}
BT::NodeStatus CreateMTCMoveToPose::tick()
{
  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveTo> stage{ nullptr };
  auto node_status = CreateMTCMoveToBase::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  // Set goal
  std::shared_ptr<geometry_msgs::PoseStamped> pose{ nullptr };
  if(!getInput(kPortGoal, pose))
    return NodeStatus::FAILURE;

  stage->setGoal(*pose);

  // Upcast to base class
  MTC::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveToPose::providedPorts()
{
  auto port_lists = CreateMTCMoveToBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<geometry_msgs::PoseStamped>>(kPortGoal, "move link to a given pose"));

  return port_lists;
}

CreateMTCMoveToPoint::CreateMTCMoveToPoint(const std::string& name,
                                         const BT::NodeConfig& config)
  : CreateMTCMoveToBase(name, config)
{}
BT::NodeStatus CreateMTCMoveToPoint::tick()
{
  // Build stage
  std::shared_ptr<moveit::task_constructor::stages::MoveTo> stage{ nullptr };
  auto node_status = CreateMTCMoveToBase::tick(stage);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;

  // Set goal
  std::shared_ptr<geometry_msgs::PointStamped> point{ nullptr };
  if(!getInput(kPortGoal, point))
    return NodeStatus::FAILURE;

  stage->setGoal(*point);

  // Upcast to base class
  MTC::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCMoveToPoint::providedPorts()
{
  auto port_lists = CreateMTCMoveToBase::providedPorts();
  port_lists.emplace(BT::InputPort<std::shared_ptr<geometry_msgs::PointStamped>>(kPortGoal, "move to specified point, keeping current orientation"));

  return port_lists;
}



//question : faut tu que je me fasse une node Geometry msg avant?

//pose
//move to pointStaped
//moveToJoints
// name joint position
//hand open pose (string) (joint pose stamped)