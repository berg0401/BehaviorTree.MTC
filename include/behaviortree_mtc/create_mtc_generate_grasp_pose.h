#pragma once

#include <behaviortree_cpp/bt_factory.h>

namespace bt_mtc
{
class CreateMTCGenerateGraspPose : public BT::SyncActionNode
{
public:
  CreateMTCGenerateGraspPose(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};
}  // namespace bt_mtc
