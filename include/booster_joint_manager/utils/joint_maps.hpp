#pragma once

#include <array>
#include <cstddef>
#include <string_view>

#include "booster/robot/b1/b1_api_const.hpp"

namespace booster_joint_manager
{

namespace b1 = booster::robot::b1;

inline constexpr std::array<b1::JointIndex, b1::kJointCnt> kAllJoints = {
  b1::JointIndex::kHeadYaw,
  b1::JointIndex::kHeadPitch,
  b1::JointIndex::kLeftShoulderPitch,
  b1::JointIndex::kLeftShoulderRoll,
  b1::JointIndex::kLeftElbowPitch,
  b1::JointIndex::kLeftElbowYaw,
  b1::JointIndex::kRightShoulderPitch,
  b1::JointIndex::kRightShoulderRoll,
  b1::JointIndex::kRightElbowPitch,
  b1::JointIndex::kRightElbowYaw,
  b1::JointIndex::kWaist,
  b1::JointIndex::kLeftHipPitch,
  b1::JointIndex::kLeftHipRoll,
  b1::JointIndex::kLeftHipYaw,
  b1::JointIndex::kLeftKneePitch,
  b1::JointIndex::kCrankUpLeft,
  b1::JointIndex::kCrankDownLeft,
  b1::JointIndex::kRightHipPitch,
  b1::JointIndex::kRightHipRoll,
  b1::JointIndex::kRightHipYaw,
  b1::JointIndex::kRightKneePitch,
  b1::JointIndex::kCrankUpRight,
  b1::JointIndex::kCrankDownRight,
};

inline constexpr std::array<float, b1::kJointCnt> kInitJointPositions = {
  0.00F, 0.00F,
  0.10F, -1.50F, 0.00F, -0.20F,
  0.10F, 1.50F, 0.00F, 0.20F,
  0.00F,
  -0.20F, 0.00F, 0.00F, 0.40F, 0.20F, 0.14F,
  -0.20F, 0.00F, 0.00F, 0.40F, 0.20F, 0.14F,
};

inline constexpr std::array<float, b1::kJointCnt> kDefaultJointKps = {
  5.0F, 5.0F,
  40.0F, 50.0F, 20.0F, 10.0F,
  40.0F, 50.0F, 20.0F, 10.0F,
  100.0F,
  350.0F, 350.0F, 180.0F, 350.0F, 550.0F, 550.0F,
  350.0F, 350.0F, 180.0F, 350.0F, 550.0F, 550.0F,
};

inline constexpr std::array<float, b1::kJointCnt> kDefaultJointKds = {
  0.1F, 0.1F,
  0.5F, 1.5F, 0.2F, 0.2F,
  0.5F, 1.5F, 0.2F, 0.2F,
  5.0F,
  7.5F, 7.5F, 3.0F, 5.5F, 1.5F, 1.5F,
  7.5F, 7.5F, 3.0F, 5.5F, 1.5F, 1.5F,
};

inline constexpr std::array<b1::JointIndex, 8> kArmJoints = {
  b1::JointIndex::kLeftShoulderPitch,
  b1::JointIndex::kLeftShoulderRoll,
  b1::JointIndex::kLeftElbowPitch,
  b1::JointIndex::kLeftElbowYaw,
  b1::JointIndex::kRightShoulderPitch,
  b1::JointIndex::kRightShoulderRoll,
  b1::JointIndex::kRightElbowPitch,
  b1::JointIndex::kRightElbowYaw,
};

std::string_view joint_name(b1::JointIndex joint);
std::size_t joint_to_index(b1::JointIndex joint);
bool is_arm_joint(b1::JointIndex joint);

}  // namespace booster_joint_manager
