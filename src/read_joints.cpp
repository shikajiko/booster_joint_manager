#include "booster_joint_manager/utils/joint_maps.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_state.hpp"

#include <vector>
#include <rclcpp>

class ReadJointProvider
{
public:
  ReadJointProvider(rclcpp::Node node, bool print_all, std::vector<JointIndex> joints) 
  {
    joint_state_subscriber = node->create_subscription<booster_interface::msg::LowState>(
      "/low_state",
      10,
      [this](booster_interface::msg::LowState::SharedPtr msg) {
        if (print_all) {
          print_all_joint_info(msg->motor_states_serial);
        } else {
          for (auto joint : joints) {
            print_all_joint_info(joint, msg->motor_state_serial[joint]);
          }
        }
      });
  }

private:
  rclcpp::Subscriber<booster_interface::msg::LowState> low_state_subscriber; 
  void print_joint_info(JointIndex joint, booster_interface::msg::MotorState state);
  void print_all_joint_info(booster_interface::msg::MotorState states[]);  
}

void ReadJointProvider::print_joint_info(JointIndex joint, booster_interface::msg::MotorState state)
{
  RCLCPP_INFO(
    node->get_logger(),
    "joint %d (%.*s): q=%.4f dq=%.4f ddq=%.4f tau_est=%.4f",
    static_cast<int>(joint),
    static_cast<int>(joint_name(joint).size()),
    joint_name(joint).data(),
    state.q,
    state.dq,
    state.ddq,
    state.tau_est);
}

void ReadJointProvider::print_all_joint_info(booster_interface::msg::MotorState states[])
{
  for (const auto joint : kAllJoints) {
    print_joint_info(joint, state[joint]);
  }
}
