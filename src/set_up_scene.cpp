#include <behaviortree_mtc/set_up_scene.h>
#include <behaviortree_mtc/shared_to_unique.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

using namespace BT;
using namespace bt_mtc;

namespace
{
constexpr auto kPortPlanningSceneInterface = "planning_scene_interface";
constexpr auto kPortCollisionObject = "collision_object";

}  // namespace

SetUpScene::SetUpScene(const std::string& name,
                       const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus SetUpScene::tick()
{
  moveit::planning_interface::PlanningSceneInterfacePtr psi;
  std::shared_ptr<moveit_msgs::CollisionObject> object;

  //INPUTS
  if(!getInput(kPortPlanningSceneInterface, psi) ||
     !getInput(kPortCollisionObject, object))
    return NodeStatus::FAILURE;
  //APPLY OBJECT
  if(!psi->applyCollisionObject(*object))
    return NodeStatus::FAILURE;
  return NodeStatus::SUCCESS;
}

BT::PortsList SetUpScene::providedPorts()
{
  return {
    BT::InputPort<moveit::planning_interface::PlanningSceneInterfacePtr>(kPortPlanningSceneInterface),
    BT::InputPort<std::shared_ptr<moveit_msgs::CollisionObject>>(kPortCollisionObject),
  };
}
