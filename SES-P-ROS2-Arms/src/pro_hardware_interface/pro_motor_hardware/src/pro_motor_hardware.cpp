#include "pro_motor_hardware/pro_motor_hardware.hpp"
#include <unistd.h>
#include "pro_motor_hardware/pro.h"
#include <string.h>
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace pro_motor_hardware
{

namespace {
  constexpr double RADIAN_TO_HUNDREDTH_DEGREE = 5729.578; // (180/π) * 100
  constexpr double HUNDREDTH_DEGREE_TO_RADIAN = 0.000174533; // (π/180) / 100
}

CallbackReturn ProMotorHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Get parameters from hardware info
  port_name_ = info_.hardware_parameters["port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baudrate"]);
  prefix_ = info_.hardware_parameters["prefix"];

  // Initialize vectors with the correct size
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  motors_.resize(info_.joints.size(), nullptr);
  joint_ids_.resize(info_.joints.size(), 0);
  joint_max_speeds_.resize(info_.joints.size(), 0);
  joint_accelerations_.resize(info_.joints.size(), 0);

  // Get parameters from URDF
  for (size_t i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joint_max_speeds_[i] = std::stoi(info_.joints[i].parameters.at("max_speed"));
    joint_accelerations_[i] = std::stoi(info_.joints[i].parameters.at("acceleration"));
  }

  // Register interfaces
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Register state interfaces
    state_interfaces_.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, 
        hardware_interface::HW_IF_POSITION, 
        &hw_positions_[i]));
    state_interfaces_.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, 
        hardware_interface::HW_IF_VELOCITY, 
        &hw_velocities_[i]));

    // Register command interfaces
    command_interfaces_.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, 
        hardware_interface::HW_IF_POSITION, 
        &hw_commands_[i]));
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ProMotorHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ProMotorHardware"), "Configuring hardware interface");
  
  // Verify parameters
  RCLCPP_INFO(rclcpp::get_logger("ProMotorHardware"), 
              "Using port: %s at baud rate: %d", 
              port_name_.c_str(), baud_rate_);

  if (port_name_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), "Serial port name not specified");
    return CallbackReturn::ERROR;
  }

  // Check if serial port exists
  if (access(port_name_.c_str(), F_OK) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                 "Serial port %s does not exist", port_name_.c_str());
    return CallbackReturn::ERROR;
  }

  // Initialize motor communication
  RCLCPP_INFO(rclcpp::get_logger("ProMotorHardware"), "Initializing pro bus...");
  if (!pro_init_bus(port_name_.c_str(), baud_rate_)) {
    RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                 "Failed to initialize pro bus: %s", strerror(errno));
    return CallbackReturn::ERROR;
  }

  // Initialize each motor
  for (size_t i = 0; i < info_.joints.size(); i++) {
    usleep(1000);

    // Get the motor ID from the parameters (starts from 1)
    int motor_id = joint_ids_[i];
    
    motors_[i] = pro_create(motor_id);  // Use the actual motor ID, not the array index
    if (!motors_[i]) {
      RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                   "Failed to create motor instance for joint %zu (ID: %d)", i, motor_id);
      cleanup();
      return CallbackReturn::ERROR;
    }

    if(!pro_set_color_led(motors_[i], PRO_LED_Green)) {
      RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                   "Failed to set LED color for motor %zu: %s", i, strerror(errno));
      return CallbackReturn::ERROR;
    }

    // Test communication and set direction for each motor
    errno = 0;
    int test_position = pro_get_position(motors_[i]);
    if (test_position == -1 && errno != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                   "Failed to communicate with motor %zu (ID: %d): %s", 
                   i, motor_id, strerror(errno));
      cleanup();
      return CallbackReturn::ERROR;
    }

    if (!pro_set_max_speed(motors_[i], joint_max_speeds_[i] * 100)) {
      RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                   "Failed to set max speed for motor %zu: %s", i, strerror(errno));
      cleanup();
      return CallbackReturn::ERROR;
    }

    if (!pro_set_acceleration(motors_[i], joint_accelerations_[i] * 100)) {
      RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                   "Failed to set angular acceleration for motor %zu: %s", i, strerror(errno));
      cleanup();
      return CallbackReturn::ERROR;
    }

    if (!pro_set_deceleration(motors_[i], joint_accelerations_[i] * 100)) {
      RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                   "Failed to set angular deceleration for motor %zu: %s", i, strerror(errno));
      cleanup();
      return CallbackReturn::ERROR;
    }
  }

  is_configured_ = true;
  RCLCPP_INFO(rclcpp::get_logger("ProMotorHardware"), "Hardware interface configured successfully");
  return CallbackReturn::SUCCESS;
}

void ProMotorHardware::cleanup()
{
  for (auto& motor : motors_) {
    if (motor) {
      pro_destroy(motor);
      motor = nullptr;
    }
  }
  pro_close_bus();
  is_configured_ = false;
}

CallbackReturn ProMotorHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ProMotorHardware"), "Activating hardware interface");
  
  if (!is_configured_) {
    RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                 "Cannot activate: hardware interface not configured");
    return CallbackReturn::ERROR;
  }

  // Set LED to blue (3) for session for each motor
  for (size_t i = 0; i < motors_.size(); i++) {
    usleep(1000);
    if (!pro_set_color_led(motors_[i], PRO_LED_Blue)) {
      RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                   "Failed to set LED color for motor %zu: %s", i, strerror(errno));
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ProMotorHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ProMotorHardware"), "Deactivating hardware interface");
  
  // Hold position for each motor
  for (size_t i = 0; i < motors_.size(); i++) {
    usleep(1000);
    if (!pro_hold(motors_[i])) {
      RCLCPP_WARN(rclcpp::get_logger("ProMotorHardware"), 
                  "Failed to hold motor %zu position", i);
    }
  }
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn ProMotorHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ProMotorHardware"), "Cleaning up hardware interface");
  
  if (!is_configured_) {
    RCLCPP_INFO(rclcpp::get_logger("ProMotorHardware"), 
                "Hardware interface already cleaned up");
    return CallbackReturn::SUCCESS;
  }

  // Reset each motor to default parameters
  for (size_t i = 0; i < motors_.size(); i++) {
    if (motors_[i]) {
      // Reset motor parameters if needed
      if (!pro_reset(motors_[i])) {
        RCLCPP_WARN(rclcpp::get_logger("ProMotorHardware"), 
                    "Failed to reset motor %zu parameters", i);
      }
    }
  }

  cleanup();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ProMotorHardware::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ProMotorHardware"), "Shutting down hardware interface");
  
  // Set motors to limp mode before shutdown
  for (size_t i = 0; i < motors_.size(); i++) {
    usleep(1000);
    if (motors_[i] && !pro_limp(motors_[i])) {
      RCLCPP_WARN(rclcpp::get_logger("ProMotorHardware"), 
                  "Failed to limp motor %zu", i);
    }
  }
  
  cleanup();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ProMotorHardware::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), "Hardware interface error");
  
  // Only reset motors that are in error state
  for (size_t i = 0; i < motors_.size(); i++) {
    if (!motors_[i]) continue;

    errno = 0;
    int status = pro_get_status(motors_[i]);
    if (status == -1 && errno != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                   "Failed to get status from motor %zu during error handling: %s", 
                   i, strerror(errno));
      continue;
    }

    if (status == PRO_StatusError) {
      if (!pro_reset(motors_[i])) {
        RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                     "Failed to reset motor %zu during error handling", i);
      } else {
        RCLCPP_INFO(rclcpp::get_logger("ProMotorHardware"), 
                    "Successfully reset motor %zu after error", i);
      }
    }
  }
  
  cleanup();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ProMotorHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < motors_.size(); i++) {
    usleep(1000);
    
    // Read position in 0.01 degree units
    errno = 0;
    int position_hundredth_deg = pro_get_position(motors_[i]);
    if (position_hundredth_deg == -1 && errno != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                   "Failed to read position from motor %zu: %s", i, strerror(errno));
      return hardware_interface::return_type::ERROR;
    }
    // Convert to radians
    hw_positions_[i] = position_hundredth_deg * HUNDREDTH_DEGREE_TO_RADIAN;

    // Read speed in 0.01 degree/s units
    errno = 0;
    int speed_hundredth_deg = pro_get_speed(motors_[i]);
    if (speed_hundredth_deg == -1 && errno != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                   "Failed to read speed from motor %zu: %s", i, strerror(errno));
      return hardware_interface::return_type::ERROR;
    }
    // Convert to radians/s
    hw_velocities_[i] = speed_hundredth_deg * HUNDREDTH_DEGREE_TO_RADIAN;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ProMotorHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < motors_.size(); i++) {
    usleep(1000);
    
    // Check if command is within valid range (e.g., ±180 degrees)
    if (std::abs(hw_commands_[i]) > M_PI) {
      RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                   "Command out of range for motor %zu: %f radians", 
                   i, hw_commands_[i]);
      return hardware_interface::return_type::ERROR;
    }

    int position_hundredth_deg = static_cast<int>(hw_commands_[i] * RADIAN_TO_HUNDREDTH_DEGREE);
    
    // Optional: Check converted value is within motor's range
    if (std::abs(position_hundredth_deg) > 18000) {  // 180 degrees * 100
      RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                   "Converted position out of range for motor %zu: %d (0.01 deg)", 
                   i, position_hundredth_deg);
      return hardware_interface::return_type::ERROR;
    }

    if (!pro_move(motors_[i], position_hundredth_deg)) {
      RCLCPP_ERROR(rclcpp::get_logger("ProMotorHardware"), 
                   "Failed to move motor %zu to commanded position", i);
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> ProMotorHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ProMotorHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

}  // namespace pro_motor_hardware