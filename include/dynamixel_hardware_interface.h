/***************************************************************
 *   Copyright 2019 Hanson Robotics Limited. All Rights Reserved.
 *
 *   NOTICE: All information contained herein is, and remains the
 *   property of Hanson Robotics Limited.
 *
 *   Author: Riddhiman Laha
 *
 ***************************************************************
Header file for the Dynamixel Hardware Interface containing the
structure for the dynamixel parameters and the class for the
Robot Hardware Interface
*/
#if !defined(HR_DYNAMIXEL_HARDWARE_INTERFACE_H)
#define HR_DYNAMIXEL_HARDWARE_INTERFACE_H

// For dynamixel
#include "dynamixel.h"
// For ROS
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/ros.h>
// For ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <boost/scoped_ptr.hpp>

// NAMESPACES
namespace hansonrobotics
{
namespace hardware
{
namespace arm
{
namespace dynamixels
{
// STRUCTURE and CLASS DEFINITIONS
/**
 * This structure contains the parameters which might be changed like the dynamixel ids, baud rate and device name
 */
struct DynamixelMotorControlParams
{
  /**
   * \param motor_ids List of motor address ids to control. Default is {}
   * \param baud_rate is set at 1000000 for all the servos
   * \param device name can be changed accordingly depending on the OS
   */
  explicit DynamixelMotorControlParams(
      const std::vector<std::uint8_t>& ids = { 8, 10, 12, 14, 16, 18, 20 },  // Presently seven servos are being used
      const int baud = 1000000, const std::string& device = "/dev/ttyUSB0")  // Name of the device
      : motor_ids(ids),
        baud_rate(baud),
        device_name(device)
  {
  }
  std::vector<std::uint8_t> motor_ids;  // dynamixel ID's
  int baud_rate;                        // baud rate (1000000)
  std::string device_name;              // device name(/dev/tty/USB0)
};

/**
 * This class contains the functions to be used by the Dynamixel Motors for group read and write
 */

class DynamixelRobotHw : public hardware_interface::RobotHW
{
public:
  DynamixelRobotHw(const DynamixelMotorControlParams& params);
  ~DynamixelRobotHw() override;

#ifndef HR_DEBUG
private:
#endif

  DynamixelMotorControlParams params_;

  // Initialize the PortHandler Instance
  dynamixel::PortHandler* port_handler_;
  // Initialize the PacketHandler Instance
  dynamixel::PacketHandler* packet_handler_;

  // Initialize GroupsyncWrite instance
  std::unique_ptr<dynamixel::GroupSyncWrite> group_sync_write_;
  // Initialize GroupsyncRead instance for the position
  std::unique_ptr<dynamixel::GroupSyncRead> group_sync_read_p;
  // Initialize GroupsyncRead instance for the velocity
  std::unique_ptr<dynamixel::GroupSyncRead> group_sync_read_v;
  // Initialize GroupsyncRead instance for the current
  std::unique_ptr<dynamixel::GroupSyncRead> group_sync_read_c;
  // Initialize GroupsyncRead instance for the input voltage
  std::unique_ptr<dynamixel::GroupSyncRead> group_sync_read_vol;

  // To connect and register the joint state interface
  void registerInterfaces();
  
  // To stop the subscriber spinner and close the port
  void cleanup();

  // The init method for ros_control which is called to initialize the
  // the RobotHW from a non-realtime thread
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  // structure containing the members for the joints
  struct Joint
  {
    double position_command;
    double position;
    double velocity;
    double effort;

    Joint() : position_command(0), position(0), velocity(0), effort(0)
    {
    }
  } joints[7];

  // structure contanining the members for the actuators
  struct Actuator
  {
    double position_command;
    double position;
    double velocity;
    double effort;

    Actuator() : position_command(0), position(0), velocity(0), effort(0)
    {
    }
  } actuators[7];

  transmission_interface::RobotTransmissions robot_transmissions_;
  boost::scoped_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader_;
  boost::shared_ptr<ros::AsyncSpinner> subscriber_spinner_;

  // Instances for the hardware interface

  // joint state interface
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;

  // actuator state interface
  hardware_interface::ActuatorStateInterface actuator_state_interface_;
  hardware_interface::PositionActuatorInterface position_actuator_interface_;
  hardware_interface::VelocityActuatorInterface velocity_actuator_interface_;
  hardware_interface::EffortActuatorInterface effort_actuator_interface_;

  // Transmission Interfaces
  transmission_interface::ActuatorToJointStateInterface* act_to_jnt_state;
  transmission_interface::JointToActuatorPositionInterface* jnt_to_act_pos;

  // Joint Limits Interface
  joint_limits_interface::JointLimits j_limits;
  joint_limits_interface::SoftJointLimits soft_limits;
  joint_limits_interface::PositionJointSoftLimitsInterface jnt_limits_interface_;

  // Function for initializing the Handlers
  void initHandlers();

  // Function for opening the dynamixel port
  int openPort();

  // Function for setting the baudrate
  int setBaudRate();

  // Function for enabling the dynamixel torque
  void enableTorque();

  // Function for adding paramter storage
  bool addParams();

  // Function for the transmission interface
  bool transmissionInterface(ros::NodeHandle& robot_hw_nh);

  // Function to write the values to the dynamixels
  void write(const ros::Time& time, const ros::Duration& period) override;

  // Function to read the values from the dynamixels
  void read(const ros::Time& time, const ros::Duration& period) override;

  // Function to convert from radians to dynamixel value
  double rad2Value(double& joint_pos_cmds_);

  // Function to convert from dynamixel value to radians
  double value2Rad(int32_t& dxl_present_position);

  // Function to convert from rpm to radians/sec
  double rpm2Radpsec(int32_t& dxl_present_velocity);

  // Function to convert milliampere (mA) to Newton meters (Nm)
  double mamps2Nm(int16_t dxl_present_current, int32_t dxl_present_velocity, int16_t dxl_present_voltage);
};

}  // namespace dynamixels
}  // namespace arm
}  // namespace hardware
}  // namespace hansonrobotics

#endif