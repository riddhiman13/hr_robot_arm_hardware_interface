/***************************************************************
 *   Copyright 2019 Hanson Robotics Limited. All Rights Reserved.
 *
 *   NOTICE: All information contained herein is, and remains the
 *   property of Hanson Robotics Limited.
 *
 *   Author: Riddhiman Laha
 *
 ***************************************************************
Test suite for the dynamixel hardware interface
*/

// Including the header containing the class
#include "dynamixel_hardware_interface.h"
// Required if tests are using ROS
//#include <ros/ros.h>
// For Google test
#include <gtest/gtest.h>

using namespace hansonrobotics::hardware::arm::dynamixels;

// class DynamixelRobotHwTest : public ::testing::Test {
// public:
// DynamixelRobotHwTest():
// private:
// };

TEST(ArmRosControl, testCase1) {
  DynamixelMotorControlParams params;

  DynamixelRobotHw robot_hw(params);

  EXPECT_EQ(params.motor_ids.size(), 7);
  EXPECT_EQ(params.baud_rate, 1000000);
  EXPECT_EQ(params.device_name, "/dev/ttyUSB0");
}

TEST(ArmRosControl, testCase2) {
  DynamixelMotorControlParams params;

  DynamixelRobotHw robot_hw(params);

  EXPECT_EQ(robot_hw.port_handler_, nullptr);
  EXPECT_EQ(robot_hw.packet_handler_, nullptr);
  EXPECT_EQ(robot_hw.group_sync_write_, nullptr);
  EXPECT_EQ(robot_hw.group_sync_read_p, nullptr);
  EXPECT_EQ(robot_hw.group_sync_read_v, nullptr);
  EXPECT_EQ(robot_hw.group_sync_read_c, nullptr);
  EXPECT_EQ(robot_hw.group_sync_read_vol, nullptr);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}