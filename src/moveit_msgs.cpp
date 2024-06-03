#include <behaviortree_mtc/moveit_msgs.h>
#include <behaviortree_mtc/custom_types.h>

#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>

using namespace BT;
using namespace bt_mtc;

namespace
{
constexpr auto kPortPose = "pose";
constexpr auto kPortObjectName = "object_name";
constexpr auto kPortObjectReferenceFrame = "object_reference_frame";
constexpr auto kPortObjectDimensions = "object_dimensions";
constexpr auto kPortCollisionObject = "collision_object";
}  // namespace

MoveItMsgsCollisionObjectBase::MoveItMsgsCollisionObjectBase(const std::string& name,
                                                             const BT::NodeConfig& config)
  : SyncActionNode(name, config)
{}

BT::NodeStatus MoveItMsgsCollisionObjectBase::tick(std::shared_ptr<moveit_msgs::CollisionObject> object)
{
  // Retrieve inputs
  std::string object_name, object_reference_frame;
  Vector2D object_dimensions_input;
  auto pose = std::make_shared<geometry_msgs::Pose>();
  if(!getInput(kPortObjectName, object_name) ||
     !getInput(kPortObjectReferenceFrame, object_reference_frame) ||
     !getInput(kPortPose, pose))
    return NodeStatus::FAILURE;
  // Build object
  object->id = object_name;
  object->header.frame_id = object_reference_frame;
  object->primitives.resize(1);
  object->primitive_poses.push_back(*pose);
  return NodeStatus::SUCCESS;
}

BT::PortsList MoveItMsgsCollisionObjectBase::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortObjectName),
    BT::InputPort<std::string>(kPortObjectReferenceFrame),
    BT::InputPort<std::shared_ptr<geometry_msgs::Pose>>(kPortPose)
  };
}

MoveItMsgsCollisionObjectCylinder::MoveItMsgsCollisionObjectCylinder(const std::string& name,
                                                                     const BT::NodeConfig& config)
  : MoveItMsgsCollisionObjectBase(name, config)
{}

BT::NodeStatus MoveItMsgsCollisionObjectCylinder::tick()
{
  // Build object
  auto object = std::make_shared<moveit_msgs::CollisionObject>();
  auto node_status = MoveItMsgsCollisionObjectBase::tick(object);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;
  // Retrieve inputs
  Vector2D object_dimensions_input;
  std::vector<double> object_dimensions(2);
  if(!getInput(kPortObjectDimensions, object_dimensions_input))
    return NodeStatus::FAILURE;
  // Set dimensions
  object->primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  object_dimensions[0] = object_dimensions_input.x;
  object_dimensions[1] = object_dimensions_input.y;
  object->primitives[0].dimensions = object_dimensions;
  setOutput(kPortCollisionObject, object);

  return NodeStatus::SUCCESS;
}

BT::PortsList MoveItMsgsCollisionObjectCylinder::providedPorts()
{
  auto port_lists = MoveItMsgsCollisionObjectBase::providedPorts();
  port_lists.emplace(BT::InputPort<Vector2D>(kPortObjectDimensions, "1,1", "cylinder height, cylinder radius"));
  port_lists.emplace(BT::OutputPort<std::shared_ptr<moveit_msgs::CollisionObject>>(kPortCollisionObject));
  return port_lists;
}

MoveItMsgsCollisionObjectBox::MoveItMsgsCollisionObjectBox(const std::string& name,
                                                           const BT::NodeConfig& config)
  : MoveItMsgsCollisionObjectBase(name, config)
{}

BT::NodeStatus MoveItMsgsCollisionObjectBox::tick()
{
  // Build object
  auto object = std::make_shared<moveit_msgs::CollisionObject>();
  auto node_status = MoveItMsgsCollisionObjectBase::tick(object);
  if(node_status != BT::NodeStatus::SUCCESS)
    return node_status;
  // Retrieve inputs
  Vector3D object_dimensions_input;
  std::vector<double> object_dimensions(3);
  if(!getInput(kPortObjectDimensions, object_dimensions_input))
    return NodeStatus::FAILURE;
  // Set dimensions
  object->primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  object_dimensions[0] = object_dimensions_input.x;
  object_dimensions[1] = object_dimensions_input.y;
  object_dimensions[2] = object_dimensions_input.z;
  object->primitives[0].dimensions = object_dimensions;
  setOutput(kPortCollisionObject, object);

  return NodeStatus::SUCCESS;
}

BT::PortsList MoveItMsgsCollisionObjectBox::providedPorts()
{
  auto port_lists = MoveItMsgsCollisionObjectBase::providedPorts();
  port_lists.emplace(BT::InputPort<Vector3D>(kPortObjectDimensions, "1,1", "cylinder height, cylinder radius"));
  port_lists.emplace(BT::OutputPort<std::shared_ptr<moveit_msgs::CollisionObject>>(kPortCollisionObject));
  return port_lists;
}