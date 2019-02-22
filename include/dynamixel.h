/***************************************************************
 *   Copyright 2019 Hanson Robotics Limited. All Rights Reserved.
 *   
 *   NOTICE: All information contained herein is, and remains the 
 *   property of Hanson Robotics Limited.
 *
 *   Author: Riddhiman Laha
 *
 ***************************************************************
Header file for the Dynamixel servos containing the classes for 
control table addresses of the servos and the data byte length 
*/

#if !defined(HR_DYNAMIXEL_ROSCONTROL_HAL_H)
#define HR_DYNAMIXEL_ROSCONTROL_HAL_H

// For the dynamixels
#include "dynamixel_sdk/dynamixel_sdk.h"

// System headers
#include <fcntl.h>    // FILE control
#include <termios.h>  // Terminal IO
#include <stdint.h>   // Integer types

// NAMESPACES
namespace hansonrobotics
{
namespace hardware
{
namespace arm
{
namespace dynamixels
{
// Constants
/** Protocol version for the Dynamixel servos **/
const float protocol_version = 2.0;

// Lookup tables

/**
 * The Control Table is a structure of data implemented in the Dynamixel. Users can
 * read a specific data to get status of the Dynamixel with Read instruction packets,
 * and modify data and control the Dynamixel with the Write Instruction packets. The
 * control table has two areas RAM and EEPROM.
 */
enum class control_table_address : int
{
  torque_enable = 64,           ///  For enabling the torque
  led_red = 65,                 ///  For turning the led red
  goal_position = 116,          ///  For obtaining the goal position
  present_position = 132,       ///  For obtaining the present position
  goal_velocity = 104,          ///  For obtaining the goal velocity
  present_velocity = 128,       ///  For obtaining the present velocity
  goal_current = 102,           ///  For obtaining the goal current
  present_current = 126,        ///  For obtaining the present current
  present_input_voltage = 144,  ///  For obtaining the present input voltage
};

enum class data_byte_length : int
{
  goal_position = 4,     /// Data byte length for the goal position
  present_position = 4,  /// Data byte length for the present position
  present_velocity = 4,  /// Data byte length for the present velocity
  present_current = 2,   /// Data byte length for the present current and voltage
};

enum dynamix_return_codes : std::uint16_t
{
  dynamix_success = 1,  /// Return codes if needed
};

}  // namespace dynamixels
}  // namespace arm
}  // namespace hardware
}  // namespace hansonrobotics
#endif  // HR_DYNAMIXEL_ROSCONTROL_HAL_H