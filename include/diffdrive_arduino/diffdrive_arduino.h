#ifndef DIFFDRIVE_ARDUINO_REAL_ROBOT_H
#define DIFFDRIVE_ARDUINO_REAL_ROBOT_H

#include <cstring>
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "config.h"
#include "wheel.h"
#include "arduino_comms.h"

using hardware_interface::return_type;

class DiffDriveArduino : public hardware_interface::SystemInterface
{
public:
  DiffDriveArduino();

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)   override;
  
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  Config cfg_;
  ArduinoComms arduino_;

  /* 4-wheel setup - FL, FR, RL, RR
     State/command interfaces are exported for all four,
     even though the ROS2 controller commands consist of only two values (left/right).
     FL and RL receive the same command (left),
     FR and RR receive the same command (right).
     
     For odometry: The Arduino sends the average per side,
     so `fl_wheel_` and `rl_wheel_` receive the same encoder value
     (the average of FL and RL from the Arduino). The same applies to FR/RR. */
  Wheel fl_wheel_;
  Wheel fr_wheel_;
  Wheel rl_wheel_;
  Wheel rr_wheel_;

  rclcpp::Logger logger_;
  
  std::chrono::time_point<std::chrono::system_clock> time_;
};

#endif // DIFFDRIVE_ARDUINO_REAL_ROBOT_H
