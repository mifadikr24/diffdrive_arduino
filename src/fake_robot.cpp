#include "diffdrive_arduino/fake_robot.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

FakeRobot::FakeRobot()
  : logger_(rclcpp::get_logger("FakeRobot"))
{}

hardware_interface::CallbackReturn FakeRobot::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  RCLCPP_INFO(logger_, "Configuring 4WD FakeRobot...");

  time_ = std::chrono::system_clock::now();

  /* Read the wheel name from the URDF parameter - it must match the
     joint name in ros2_control.xacro and controllers.yaml */
  cfg_.front_left_wheel_name  = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.rear_left_wheel_name   = info_.hardware_parameters["rear_left_wheel_name"];
  cfg_.rear_right_wheel_name  = info_.hardware_parameters["rear_right_wheel_name"];

  /* FakeRobot does not require encoder counts for odometry
     because the position is integrated directly from the velocity command.
     The CPR value is set to 1 (rads_per_count = 2phi) as a placeholder */
  fl_wheel_.setup(cfg_.front_left_wheel_name,  1);
  fr_wheel_.setup(cfg_.front_right_wheel_name, 1);
  rl_wheel_.setup(cfg_.rear_left_wheel_name,   1);
  rr_wheel_.setup(cfg_.rear_right_wheel_name,  1);

  RCLCPP_INFO(logger_, "Finished Configuration");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FakeRobot::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.vel);
  state_interfaces.emplace_back(fl_wheel_.name, hardware_interface::HW_IF_POSITION, &fl_wheel_.pos);

  state_interfaces.emplace_back(fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.vel);
  state_interfaces.emplace_back(fr_wheel_.name, hardware_interface::HW_IF_POSITION, &fr_wheel_.pos);

  state_interfaces.emplace_back(rl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rl_wheel_.vel);
  state_interfaces.emplace_back(rl_wheel_.name, hardware_interface::HW_IF_POSITION, &rl_wheel_.pos);

  state_interfaces.emplace_back(rr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rr_wheel_.vel);
  state_interfaces.emplace_back(rr_wheel_.name, hardware_interface::HW_IF_POSITION, &rr_wheel_.pos);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FakeRobot::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.cmd);
  command_interfaces.emplace_back(fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.cmd);
  command_interfaces.emplace_back(rl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rl_wheel_.cmd);
  command_interfaces.emplace_back(rr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rr_wheel_.cmd);

  return command_interfaces;
}

hardware_interface::CallbackReturn FakeRobot::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Starting 4WD FakeRobot Controller...");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeRobot::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Stopping 4WD FakeRobot Controller...");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FakeRobot::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;

  /* Position integration based on velocity for all four wheels.
     FL and RL will have the same velocity (set by write()),
     as will FR and RR. */
  fl_wheel_.pos += fl_wheel_.vel * deltaSeconds;
  fr_wheel_.pos += fr_wheel_.vel * deltaSeconds;
  rl_wheel_.pos += rl_wheel_.vel * deltaSeconds;
  rr_wheel_.pos += rr_wheel_.vel * deltaSeconds;

  return return_type::OK;
}

hardware_interface::return_type FakeRobot::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  /* Set velocity state = velocity command (open loop, no PID).
     The diff_drive_controller writes the same command to FL+RL and FR+RR,
     so we simply pass it through directly. */
  fl_wheel_.vel = fl_wheel_.cmd;
  fr_wheel_.vel = fr_wheel_.cmd;
  rl_wheel_.vel = rl_wheel_.cmd;
  rr_wheel_.vel = rr_wheel_.cmd;

  return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(FakeRobot, hardware_interface::SystemInterface)