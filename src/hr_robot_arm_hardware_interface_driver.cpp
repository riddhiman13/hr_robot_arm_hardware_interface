/***************************************************************
 *   Copyright 2019 Hanson Robotics Limited. All Rights Reserved.
 *
 *   NOTICE: All information contained herein is, and remains the
 *   property of Hanson Robotics Limited.
 *
 *   Author: Riddhiman Laha
 *
 ***************************************************************
Driver node containing the main function
*/
//For the Dynamixel SDK
#include "dynamixel_hardware_interface.h"
//For ros_control
#include <controller_manager/controller_manager.h>

// Main function
int main(int argc, char** argv)
{
  try
  {
    ROS_INFO("starting");

    // Initiate a ros node called ArmDynamixelHardwareInterface
    ros::init(argc, argv, "ArmDynamixelsHardwareInterface");

    // NodeHandle object which represents the ROS Node ("namespace" of the controller)
    ros::NodeHandle nh;

    // NodeHandle object in the namespace from which the RobotHW should read its
    // configuration
    ros::NodeHandle robot_hw_nh;

    // Change the levels accordingly
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
      ros::console::notifyLoggerLevelsChanged();

    ROS_DEBUG("\n ============ We have started this program at DEBUG log level ================\n");

    using namespace hansonrobotics::hardware::arm::dynamixels;

    DynamixelMotorControlParams params;

    // Create an object of the Hardware Interface class
    DynamixelRobotHw robot_hw(params);

    // The init method is called in non-realtime to initialize the RobotHW
    robot_hw.init(nh, robot_hw_nh);

    // Create controller manager instance
    controller_manager::ControllerManager cm(&robot_hw);

    // Non-blocking 4-threaded service of call-backs
    ros::AsyncSpinner spinner(4);
    // Spins asynchronously until stopped
    spinner.start();

    // The rate object maintains a 50 Hz loop rate
    ros::Rate loop_rate(50);

    ros::Time last_time = ros::Time::now();

    while (ros::ok())
    {
      loop_rate.sleep();

      // Get the current time as a ros::Time instance
      ros::Time current_time = ros::Time::now();

      // Calculate the period of time elapsed
      ros::Duration elapsed_time = current_time - last_time;

      last_time = current_time;

      // Read the values from the Dynamixels
      robot_hw.read(current_time, elapsed_time);

      // Update the controllers based on time since last iteration
      cm.update(current_time, elapsed_time);

      // Write the commanded values to the servos
      robot_hw.write(current_time, elapsed_time);
    }

    robot_hw.cleanup();
  }
  catch (...)
  {
    ROS_ERROR("Unhandled exception !");
    return -1;
  }

  return 0;
}
