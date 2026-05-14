#include "booster_joint_manager/joint_manager/joint_manager.hpp"

#include <algorithm>
#include <cmath>

namespace booster_joint_manager
{

void JointManager::handle_set_joints(const std::vector<JointCommandTarget> & targets)
{
  booster_interface::msg::LowState low_state;
  if (targets.empty() || !get_low_state(low_state)) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex);
  if (command_running || should_publish_set_torque) {
    return;
  }

  target_command.clear();
  active_command.clear();
  const auto & serial_states = low_state.motor_state_serial;
  for (const auto & target : targets) {
    const auto index = joint_to_index(target.joint);
    if (index >= b1::kJointCnt) {
      continue;
    }

    const auto current_q = index < serial_states.size() ? serial_states[index].q : 0.0F;
    target_command.push_back(target);
    active_command.push_back(
      JointCommandTarget{
        target.joint,
        current_q,
        target.velocity,
        kInactiveJointWeight,
      });
  }

  command_running = !active_command.empty();
}

void JointManager::handle_set_torques(const std::vector<b1::JointIndex> & joints, bool torque_enable)
{
  booster_interface::msg::LowState low_state;
  if (joints.empty() || !get_low_state(low_state)) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex);
  if (command_running || should_publish_set_torque) {
    return;
  }

  torque_command = construct_set_torque_command(low_state, joints, torque_enable);
  should_publish_set_torque = true; 
}

void JointManager::set_init_position(bool arm_only)
{
  booster_interface::msg::LowState low_state;
  if (!get_low_state(low_state)) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex);
  if (command_running || should_publish_set_torque) {
    return;
  }

  target_command.clear();
  active_command.clear();

  const auto & serial_states = low_state.motor_state_serial;

  for (const auto joint : kAllJoints) {
    const auto index = joint_to_index(joint);
    const auto current_q = index < serial_states.size() ? serial_states[index].q : 0.0F;
    const auto target_q = index < kInitJointPositions.size() ? kInitJointPositions[index] : current_q;
    const auto should_control_joint = !arm_only || is_arm_joint(joint);

    if (should_control_joint) {
      target_command.push_back(
        JointCommandTarget{
          joint,
          target_q,
          1.0F,
          kActiveJointWeight,
        });
      active_command.push_back(
        JointCommandTarget{
          joint,
          current_q,
          1.0F,
          kInactiveJointWeight,
        });
    }
  }

  command_running = !active_command.empty();
}

void JointManager::maintain_current_pose()
{
  booster_interface::msg::LowState low_state;
  if (!get_low_state(low_state)) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex);
  if (command_running || should_publish_set_torque) {
    return;
  }

  target_command.clear();
  active_command.clear();

  const auto & serial_states = low_state.motor_state_serial;

  for (const auto joint : kAllJoints) {
    const auto index = joint_to_index(joint);
    const auto current_q = index < serial_states.size() ? serial_states[index].q : 0.0F;

    target_command.push_back(
      JointCommandTarget{
        joint,
        current_q,
        1.0F,
        kActiveJointWeight,
      });
    active_command.push_back(
      JointCommandTarget{
        joint,
        current_q,
        1.0F,
        kInactiveJointWeight,
      });
  }

  command_running = !active_command.empty();
}

bool JointManager::tick_command(booster_interface::msg::LowCmd & cmd)
{
  std::lock_guard<std::mutex> lock(mutex);

  if (should_publish_set_torque) {
    should_publish_set_torque = false;
    cmd = torque_command;
    return true;
  }

  if (!command_running || !low_state_received) {
    return false;
  }

  bool command_reached = true;

  for (std::size_t i = 0; i < active_command.size() && i < target_command.size(); ++i) {
    auto & active_target = active_command[i];
    const auto & target = target_command[i];

    if (active_target.joint != target.joint) {
      continue;
    }

    const auto velocity_scale = std::abs(target.velocity) > 0.0F ? std::abs(target.velocity) : 1.0F;
    const auto max_joint_delta = std::clamp(kBaseJointStep * velocity_scale, 0.0F, kMaxJointStep);

    const auto delta_q = target.position - active_target.position;
    const auto joint_step = std::clamp(delta_q, -max_joint_delta, max_joint_delta);
    const auto weight_step = std::clamp(target.weight - active_target.weight, -kWeightMargin, kWeightMargin);

    active_target.position += joint_step;
    active_target.weight += weight_step;

    if (std::abs(delta_q) > max_joint_delta || std::abs(target.weight - active_target.weight) > kWeightMargin) {
      command_reached = false;
    }
  }

  cmd = construct_joint_command(current_low_state, active_command);
  if (command_reached) {
    command_running = false;
  }

  return true;
}

void JointManager::update_low_state(const booster_interface::msg::LowState & low_state)
{
  std::lock_guard<std::mutex> lock(mutex);
  current_low_state = low_state;
  low_state_received = true;
}

bool JointManager::get_joint_state(b1::JointIndex joint, booster_interface::msg::MotorState & joint_state) const 
{
  const auto index = joint_to_index(joint);
  std::lock_guard<std::mutex> lock(mutex);
  if (!low_state_received || index >= current_low_state.motor_state_serial.size()) {
    return false;
  }

  joint_state = current_low_state.motor_state_serial[index];
  return true;
}

bool JointManager::get_low_state(booster_interface::msg::LowState & low_state) const
{
  std::lock_guard<std::mutex> lock(mutex);
  if (!low_state_received) {
    return false;
  }

  low_state = current_low_state;
  return true;
}

bool JointManager::has_low_state() const
{
  std::lock_guard<std::mutex> lock(mutex);
  return low_state_received;
}

}  // namespace booster_joint_manager
