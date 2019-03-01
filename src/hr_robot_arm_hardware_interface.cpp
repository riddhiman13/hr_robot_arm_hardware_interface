/***************************************************************
 *   Copyright 2019 Hanson Robotics Limited. All Rights Reserved.
 *
 *   NOTICE: All information contained herein is, and remains the
 *   property of Hanson Robotics Limited.
 *
 *   Author: Riddhiman Laha
 *
 ***************************************************************
Hardware interface for Hanson Robotics Sophia Arm
which currently uses Dynamixel servos
*/
// For the header
#include "dynamixel_hardware_interface.h"
// For the ros_control controller manager
#include <controller_manager/controller_manager.h>
#include <stdexcept>
#include <urdf/model.h>

// NAMESPACES
namespace hansonrobotics {
namespace hardware {
namespace arm {
namespace dynamixels {
using namespace transmission_interface;

// PUBLIC METHODS for DynamixelRobotHw
DynamixelRobotHw::DynamixelRobotHw(const DynamixelMotorControlParams &params)
    : params_(params), port_handler_(nullptr), packet_handler_(nullptr),
      group_sync_write_(nullptr), group_sync_read_p(nullptr),
      group_sync_read_v(nullptr), group_sync_read_c(nullptr),
      group_sync_read_vol(nullptr) {
  registerInterfaces();
  ROS_DEBUG("Constructor finished");
}

DynamixelRobotHw::~DynamixelRobotHw() {}

// PRIVATE METHODS for DynamixelRobotHw

// To connect and register the joint state interface
void DynamixelRobotHw::registerInterfaces() {
  ROS_DEBUG("register interfaces has been called");

  // Reading the actuator names from the configuration file loaded in the
  // parameter server
  XmlRpc::XmlRpcValue act_list;
  ros::NodeHandle nh;
  nh.getParam("/actuators", act_list);

  std::string actuator_names[act_list.size()];

  for (int i = 0; i < act_list.size(); ++i) {
    XmlRpc::XmlRpcValue sublist = act_list[i];
    if (sublist["name"].size() > 0 && sublist["type"].size() > 0 &&
        sublist["model"].size() > 0) {
      std::string name = sublist["name"];
      actuator_names[i] = name;
    }
  }

  for (unsigned int i{0}; i < (sizeof(actuators) / sizeof(actuators[0])); ++i) {
    hardware_interface::ActuatorStateHandle actuator_state_handle(
        actuator_names[i], &actuators[i].position, &actuators[i].velocity,
        &actuators[i].effort);
    ROS_INFO_STREAM("The Actuator is: " << actuator_names[i]);
    actuator_state_interface_.registerHandle(actuator_state_handle);
    hardware_interface::ActuatorHandle actuator_handle(
        actuator_state_handle, &actuators[i].position_command);
    position_actuator_interface_.registerHandle(actuator_handle);
    velocity_actuator_interface_.registerHandle(actuator_handle);
    effort_actuator_interface_.registerHandle(actuator_handle);
  }
  registerInterface(&actuator_state_interface_);
  registerInterface(&position_actuator_interface_);
  registerInterface(&velocity_actuator_interface_);
  registerInterface(&effort_actuator_interface_);
}

// To stop the subscriber spinner and close the port
void DynamixelRobotHw::cleanup() {
  subscriber_spinner_->stop();
  port_handler_->closePort();
}

// Function for initializing the Handlers
void DynamixelRobotHw::initHandlers() {
  port_handler_ =
      dynamixel::PortHandler::getPortHandler(params_.device_name.c_str());

  packet_handler_ =
      dynamixel::PacketHandler::getPacketHandler(protocol_version);

  group_sync_write_ = std::make_unique<
      dynamixel::GroupSyncWrite>( // For writing the goal position
      port_handler_, packet_handler_,
      static_cast<std::uint16_t>(control_table_address::goal_position),
      static_cast<std::uint16_t>(data_byte_length::goal_position));

  group_sync_read_p = std::make_unique<
      dynamixel::GroupSyncRead>( // For reading the present position
      port_handler_, packet_handler_,
      static_cast<std::uint16_t>(control_table_address::present_position),
      static_cast<std::uint16_t>(data_byte_length::present_position));

  group_sync_read_v = std::make_unique<
      dynamixel::GroupSyncRead>( // For reading the present velocity
      port_handler_, packet_handler_,
      static_cast<std::uint16_t>(control_table_address::present_velocity),
      static_cast<std::uint16_t>(data_byte_length::present_velocity));

  group_sync_read_c = std::make_unique<
      dynamixel::GroupSyncRead>( // For reading the present current
      port_handler_, packet_handler_,
      static_cast<std::uint16_t>(control_table_address::present_current),
      static_cast<std::uint16_t>(data_byte_length::present_current));

  group_sync_read_vol = std::make_unique<
      dynamixel::GroupSyncRead>( // For reading the input voltage
      port_handler_, packet_handler_,
      static_cast<std::uint16_t>(control_table_address::present_input_voltage),
      static_cast<std::uint16_t>(data_byte_length::present_current));
}

// Function for opening the dynamixel port
int DynamixelRobotHw::openPort() { return (port_handler_->openPort()); }

// Function for setting the baudrate
int DynamixelRobotHw::setBaudRate() {
  return (port_handler_->setBaudRate(params_.baud_rate));
}

// Function for enabling the dynamixel torque
void DynamixelRobotHw::enableTorque() {
  uint8_t dxl_error{0};
  int dxl_comm_result = COMM_TX_FAIL;
  for (auto id : params_.motor_ids) {
    dxl_comm_result = packet_handler_->write1ByteTxRx(
        port_handler_, id,
        static_cast<uint16_t>(control_table_address::torque_enable), 1,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS) {
      ROS_ERROR("Error connecting to the dynamixels");
    }

    else if (dxl_error != 0) {
      ROS_ERROR("Error connecting to the dynamixels");
    } else {
      ROS_INFO_STREAM("Dynamixel " << static_cast<int>(id)
                                   << " connected successfully");
    }
  }
}

// Function for adding paramter storage in the servos
bool DynamixelRobotHw::addParams() {
  bool dxl_addparam_result = false;
  for (auto id : params_.motor_ids) {
    dxl_addparam_result = group_sync_read_p->addParam(id);
    dxl_addparam_result = group_sync_read_v->addParam(id);
    dxl_addparam_result = group_sync_read_c->addParam(id);
    dxl_addparam_result = group_sync_read_vol->addParam(id);
  }
  return dxl_addparam_result;
}

// Function for the joint limits interface
bool DynamixelRobotHw::jointlimitsInterface() {

  urdf::Model urdf;
  if (!urdf.initParam("/robot_description"))
    ROS_ERROR(
        "DynamixelRobotHw::jointlimitsInterface: /robot_description missing.");

  for (auto &joint : urdf.joints_) {
    const auto name = joint.first;
    ROS_INFO_STREAM("Joint name obtained from the URDF file: " << name);
    boost::shared_ptr<const urdf::Joint> urdf_joint_0 = urdf.getJoint(name);
    if (urdf_joint_0 == nullptr)
      ROS_ERROR_STREAM("DynamixelRobotHw::jointlimitsInterface:: cannot load "
                       << name);
    ROS_INFO_STREAM("Joint type: " << joint.second->type);

    // For revolute joints only
    if (joint.second->type == 1) {
      const bool urdf_limits_ok0 = getJointLimits(urdf_joint_0, j_limits_0);

      if (urdf_limits_ok0 != 1)
        ROS_ERROR("Error loading the joint limits from URDF");

      const bool urdf_soft_limits_ok0 =
          getSoftJointLimits(urdf_joint_0, soft_limits);

      if (urdf_soft_limits_ok0 != 1)
        ROS_ERROR("Error loading the soft joint limits from URDF");

      hardware_interface::PositionJointInterface *pos_jnt_iface =
          this->get<hardware_interface::PositionJointInterface>();

      joint_handle_0 = pos_jnt_iface->getHandle(name);

      joint_limits_interface::PositionJointSoftLimitsHandle handle0(
          joint_handle_0, j_limits_0, soft_limits);

      jnt_limits_interface_.registerHandle(handle0);
    }
  }
  return true;
}

// Function for the transmission interface
bool DynamixelRobotHw::transmissionInterface(ros::NodeHandle &robot_hw_nh) {
  // Initializing the transmission loader
  try {
    transmission_loader_.reset(
        new transmission_interface::TransmissionInterfaceLoader(
            this, &robot_transmissions_));
  } catch (const std::invalid_argument &ex) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. "
                     << ex.what());
    return false;
  } catch (const pluginlib::LibraryLoadException &ex) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. "
                     << ex.what());
    return false;
  } catch (...) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
    return false;
  }

  // Obtaining the URDF from parameter server
  std::string robot_description;
  robot_hw_nh.getParam("/robot_description", robot_description);

  // Perform actual transmission loading
  if (!transmission_loader_->load(robot_description)) {
    return false;
  }

  // Obtaining the transmission interface for propagating the actuator states
  act_to_jnt_state = robot_transmissions_.get<ActuatorToJointStateInterface>();

  // Obtaining the transmission interface for propagating the joint position
  // commands
  jnt_to_act_pos = robot_transmissions_.get<JointToActuatorPositionInterface>();

  return true;
}

// Function to convert from radians to dynamixel value
std::uint16_t DynamixelRobotHw::rad2Value(double position_command) {
  constexpr int dyna_max = 4096;
  constexpr float rad_to_encoder = dyna_max / (2 * M_PI);
  constexpr int centre = dyna_max / 2;

  position_command = centre + round(position_command * rad_to_encoder);

  if (position_command > dyna_max)
    position_command = dyna_max;

  ROS_DEBUG("rad2Value: position_command : %f (should be between 0 and 4095)",
            position_command);

  return static_cast<std::uint16_t>(position_command);
}

// Function to propagate joint commands to the dynamixels
void DynamixelRobotHw::write(const ros::Time &time,
                             const ros::Duration &period) {
  ROS_DEBUG_STREAM("write called");

  ROS_DEBUG_STREAM(" (Before Enforcing Limits) In DynamixelRobotHw::write "
                   "Command given for: "
                   << joint_handle_0.getName() << " is "
                   << joint_handle_0.getCommand());

  // Enforcing joint limits for all the registered joint handles
  jnt_limits_interface_.enforceLimits(period);

  ROS_DEBUG_STREAM(" (After Enforcing Limits) In DynamixelRobotHw::write "
                   "Command given for: "
                   << joint_handle_0.getName() << " is "
                   << joint_handle_0.getCommand());

  // Propagate the joint commands (positions) to actuators
  jnt_to_act_pos->propagate();

  // Sending actuator command to the hardware
  bool addparam_result = false;
  int raw_value{0};
  int i{0};

  for (auto id : params_.motor_ids) {

    raw_value = rad2Value(this->actuators[i].position_command);

    uint8_t param_goal_position[] = {
            [0] = DXL_LOBYTE(DXL_LOWORD(raw_value)),
            [1] = DXL_HIBYTE(DXL_LOWORD(raw_value)),
            [2] = DXL_LOBYTE(DXL_HIWORD(raw_value)),
            [3] = DXL_HIBYTE(DXL_HIWORD(raw_value)),
    };

    addparam_result = group_sync_write_->addParam(id, param_goal_position);
    if (addparam_result != 1)
      ROS_ERROR("Error in communication");
    bool status = group_sync_write_->txPacket();
    if (status != 0)
      ROS_ERROR("Error in communication");
    group_sync_write_->clearParam();
    ++i;
  }
}

// Function to convert from dynamixel value to radians
double DynamixelRobotHw::value2Rad(int32_t &dxl_present_position) {
  ROS_DEBUG("value2Rad (read): %d", dxl_present_position);
  constexpr int dyna_max = 4096;
  constexpr int centre = dyna_max / 2;
  constexpr float encoder_to_rad = (2 * M_PI) / dyna_max;

  double result = dxl_present_position;
  if (dxl_present_position <= centre)
    result = (result - centre) * encoder_to_rad;

  if (dxl_present_position > centre)
    result = (centre - result) * encoder_to_rad;

  if (dxl_present_position > dyna_max) 
    result = dyna_max * encoder_to_rad;

  // ROS_DEBUG("value2Rad (read) pre rad: %f", result);
  // result *= encoder_to_rad;
  // ROS_DEBUG("value2Rad (read) rad: %f", result);
  return result;
}

// Function to convert from rpm to radians/sec
double DynamixelRobotHw::rpm2Radpsec(int32_t &dxl_present_velocity) {
  const double constant1 = 0.229;
  const double constant2 = 0.104719;
  return (dxl_present_velocity * constant1 * constant2);
}

// Function to convert milliampere (mA) to Newton meters (Nm) [Torque]
double DynamixelRobotHw::mamps2Nm(int16_t dxl_present_current,
                                  int32_t dxl_present_velocity,
                                  int16_t dxl_present_voltage) {
  const double constant1 = 0.229;
  const double constant2 = 0.104719;
  const double constant3 = 3.36;
  const double efficiency = 0.2;
  double result =
      ((dxl_present_current * dxl_present_voltage * constant3 * efficiency) /
       (dxl_present_velocity * constant1 * constant2 * 1000));
  if (result > 0) {
    return result;
  } else {
    return 0;
  }
}

// Function to read the values from the dynamixels and propagate them to the
// joints
void DynamixelRobotHw::read(const ros::Time &time,
                            const ros::Duration &period) {
  ROS_DEBUG_STREAM("read called");

  // Reading actuator states from the hardware
  int status1 = group_sync_read_p->txRxPacket();
  int status2 = group_sync_read_v->txRxPacket();
  int status3 = group_sync_read_c->txRxPacket();
  int status4 = group_sync_read_vol->txRxPacket();

  if ((status1 && status2 && status3 && status4) != 0) {
    ROS_ERROR("Error in communication");
  }

  bool result = false;
  int32_t dxl_present_position{0};
  int32_t dxl_present_velocity{0};
  int32_t dxl_present_current{0};
  int32_t dxl_present_voltage{0};
  int i{0};

  for (auto id : params_.motor_ids) {
    result = group_sync_read_p->isAvailable(
        id, static_cast<uint16_t>(control_table_address::present_position),
        static_cast<uint16_t>(data_byte_length::present_position));

    result = group_sync_read_v->isAvailable(
        id, static_cast<uint16_t>(control_table_address::present_velocity),
        static_cast<uint16_t>(data_byte_length::present_position));

    result = group_sync_read_c->isAvailable(
        id, static_cast<uint16_t>(control_table_address::present_current),
        static_cast<uint16_t>(data_byte_length::present_current));

    result = group_sync_read_vol->isAvailable(
        id, static_cast<uint16_t>(control_table_address::present_input_voltage),
        static_cast<uint16_t>(data_byte_length::present_current));

    if (result != true) {
      ROS_ERROR_STREAM("Group "
                       << "Sync Failed");
    }

    dxl_present_position = group_sync_read_p->getData(
        id, static_cast<uint16_t>(control_table_address::present_position),
        static_cast<uint16_t>(data_byte_length::present_position));

    dxl_present_velocity = group_sync_read_v->getData(
        id, static_cast<uint16_t>(control_table_address::present_velocity),
        static_cast<uint16_t>(data_byte_length::present_position));

    dxl_present_current = group_sync_read_c->getData(
        id, static_cast<uint16_t>(control_table_address::present_current),
        static_cast<uint16_t>(data_byte_length::present_current));

    dxl_present_voltage = group_sync_read_vol->getData(
        id, static_cast<uint16_t>(control_table_address::present_input_voltage),
        static_cast<uint16_t>(data_byte_length::present_current));

    double position_rad = value2Rad(dxl_present_position);
    double velocity = rpm2Radpsec(dxl_present_velocity);
    double torque = mamps2Nm(dxl_present_current, dxl_present_velocity,
                             dxl_present_voltage);

    this->actuators[i].position = position_rad;
    this->actuators[i].velocity = velocity;
    this->actuators[i].effort = torque;
    ++i;
  }

  // Propagate the current actuator state to the joints
  act_to_jnt_state->propagate();
}

// The init method for ros_control
bool DynamixelRobotHw::init(ros::NodeHandle &root_nh,
                            ros::NodeHandle &robot_hw_nh) {
  initHandlers();
  openPort();
  setBaudRate();
  ROS_INFO("Before Enable torque");
  enableTorque();
  ROS_DEBUG("After Enable torque");
  addParams();
  ROS_DEBUG("After addParams");
  transmissionInterface(robot_hw_nh);
  ROS_DEBUG("After transmission interface loader");
  jointlimitsInterface();
  ROS_DEBUG("After joint limits interface");

  return true;
}

} // namespace dynamixels
} // namespace arm
} // namespace hardware
} // namespace hansonrobotics
