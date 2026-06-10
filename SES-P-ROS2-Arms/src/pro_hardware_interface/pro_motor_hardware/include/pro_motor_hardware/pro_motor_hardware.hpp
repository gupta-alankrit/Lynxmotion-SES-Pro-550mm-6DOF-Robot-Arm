#ifndef PRO_MOTOR_HARDWARE_HPP_
#define PRO_MOTOR_HARDWARE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pro_motor_hardware/pro.h"

#include <vector>
#include <string>

namespace pro_motor_hardware
{

class ProMotorHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ProMotorHardware)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void cleanup();

  // Parameters
  std::string port_name_;
  int baud_rate_;
  bool is_configured_{false};
  std::string prefix_;
  
  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  
  // Motor instances and parameters
  std::vector<PRO*> motors_;
  std::vector<int> joint_ids_;
  std::vector<int> joint_max_speeds_;
  std::vector<int> joint_accelerations_;

  // Interface vectors
  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;
};

}  // namespace pro_motor_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pro_motor_hardware::ProMotorHardware,
  hardware_interface::SystemInterface
)

#endif  // PRO_MOTOR_HARDWARE_HPP_
