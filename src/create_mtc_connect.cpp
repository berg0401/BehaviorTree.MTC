#include <behaviortree_mtc/create_mtc_connect.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/task_constructor/stages/connect.h>


using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

namespace
{
constexpr auto kPortStageName = "stage_name";
constexpr auto kPortTimeout = "timeout";
constexpr auto kPortMergeMode = "merge_mode";
constexpr auto kPortMaxDistance = "max_distance";
constexpr auto kPortGroup = "group";
constexpr auto kPortPlanner = "planner";
constexpr auto kPortStage = "stage";
}  // namespace

CreateMTCConnect::CreateMTCConnect(const std::string& name,
                                               const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreateMTCConnect::tick()
{
  // Retrieve inputs
  double timeout;
  double max_distance;
  uint8_t merge_mode;
  std::string name, group; 
  MTC::solvers::PlannerInterfacePtr planner; 
  if(!getInput(kPortTimeout, timeout) ||
     !getInput(kPortMergeMode, merge_mode) ||
     !getInput(kPortMaxDistance, max_distance) ||
     !getInput(kPortStageName, name) ||
     !getInput(kPortPlanner, planner) ||
     !getInput(kPortGroup, group))
    return NodeStatus::FAILURE;

  // Build stage
  std::shared_ptr<MTC::stages::Connect> stage{ nullptr };
  stage = {
    new MTC::stages::Connect(name, MTC::stages::Connect::GroupPlannerVector{ { group, planner } }),
    dirty::fake_deleter{}
  };
  stage->setTimeout(timeout);
  stage->setProperty("merge_mode",static_cast<MTC::stages::Connect::MergeMode>(merge_mode));
  stage->setProperty("max_distance",max_distance);

  // Upcast to base class
  MTC::StagePtr base_stage = stage;

  setOutput(kPortStage, base_stage);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreateMTCConnect::providedPorts()
{
  return {
    //Output
    BT::OutputPort<MTC::StagePtr>(kPortStage, "{connect_stage}", "Connect stage"),
    //Inputs
    BT::InputPort<double>(kPortTimeout, 1.0,"timeout"),
    BT::InputPort<double>(kPortMaxDistance,1e-4,
	                  "maximally accepted joint configuration distance between trajectory endpoint and goal state"),
    BT::InputPort<std::string>(kPortStageName,"connect"),
    BT::InputPort<uint8_t>(kPortMergeMode, 1, "0 = SEQUENTIAL, 1 = WAYPOINTS"),
    BT::InputPort<std::string>(kPortGroup),
    BT::InputPort<MTC::solvers::PlannerInterfacePtr>(kPortPlanner),
  };
}

//connects to different stage or poses. techinquement dans la demo, le robot a pas encore bouger ya juste ouvert sa main