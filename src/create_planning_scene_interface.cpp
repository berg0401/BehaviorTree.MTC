#include <behaviortree_mtc/create_planning_scene_interface.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace BT;
using namespace bt_mtc;

namespace
{
constexpr auto kPortPlanningSceneInterface = "planning_scene_interface";

}  // namespace

CreatePlanningSceneInterface::CreatePlanningSceneInterface(const std::string& name,
                                                           const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus CreatePlanningSceneInterface::tick()
{
  auto planningSceneInterface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  // Upcast to base class
  moveit::planning_interface::PlanningSceneInterfacePtr planningSceneInterface_ptr = planningSceneInterface;
  setOutput(kPortPlanningSceneInterface, planningSceneInterface_ptr);

  return NodeStatus::SUCCESS;
}

BT::PortsList CreatePlanningSceneInterface::providedPorts()
{
  return {
    BT::OutputPort<moveit::planning_interface::PlanningSceneInterfacePtr>(kPortPlanningSceneInterface, "{planning_scene_interface}"),
  };
}