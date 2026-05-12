#pragma once

#include <mutex>

#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_state.hpp"
#include "booster_joint_manager/utils/command_constructor.hpp"
#include "booster_joint_manager/utils/joint_maps.hpp"

namespace booster_joint_manager
{

class JointManager
{
public:
  JointManager() = default;

  void update_low_state(const booster_interface::msg::LowState & state);
  void handle_set_joints(const std::vector<JointCommandTarget> & targets);
  void handle_set_torques(const std::vector<b1::JointIndex> & joints, bool torque_enable);
  bool tick_command(booster_interface::msg::LowCmd & cmd);
  bool get_joint_state(b1::JointIndex joint, booster_interface::msg::MotorState & state) const;
  bool get_low_state(booster_interface::msg::LowState & state) const;
  bool has_low_state() const;

private:
  mutable std::mutex mutex;
  booster_interface::msg::LowState current_low_state;
  bool low_state_received{false};
  booster_interface::msg::LowCmd target_cmd;
  booster_interface::msg::LowCmd active_cmd;
  std::vector<JointCommandTarget> active_targets;
  bool should_publish_set_torque{false};
  bool command_running{false};
};

}  // namespace booster_joint_manager
