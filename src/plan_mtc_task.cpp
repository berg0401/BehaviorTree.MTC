#include <behaviortree_mtc/plan_mtc_task.h>
#include <ros/ros.h>

#include <moveit/task_constructor/task.h>

using namespace BT;
using namespace bt_mtc;
namespace MTC = moveit::task_constructor;

namespace
{
constexpr auto kPortTask = "task";
constexpr auto kPortMaxSolutions = "max_solutions";

}  // namespace

PlanMTCTask::PlanMTCTask(const std::string& name, const BT::NodeConfig& config)
  : ThreadedAction(name, config)
{}

BT::NodeStatus PlanMTCTask::tick()
{
  const size_t max_solutions = getInput<size_t>(kPortMaxSolutions).value();
  if(auto any_ptr = getLockedPortContent(kPortTask))
  {
    if(auto* task_ptr = any_ptr->castPtr<MTC::TaskPtr>())
    {
      try{
      auto& task = *task_ptr;
      if(task->plan(max_solutions))
      {
        return NodeStatus::SUCCESS;
      }
      }
      catch (moveit::task_constructor::InitStageException e)
      {
        ROS_ERROR_STREAM(e);
      }
      return NodeStatus::FAILURE;
    }
  }
  return NodeStatus::FAILURE;
}

BT::PortsList PlanMTCTask::providedPorts()
{
  return {
    BT::BidirectionalPort<MTC::TaskPtr>(kPortTask),
    BT::InputPort<size_t>(kPortMaxSolutions),
  };
}
