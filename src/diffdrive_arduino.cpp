#include "diffdrive_arduino/diffdrive_arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino"))
{}

hardware_interface::CallbackReturn DiffDriveArduino::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  RCLCPP_INFO(logger_, "Configuring 4WD DiffDriveArduino...");

  time_ = std::chrono::system_clock::now();

  cfg_.front_left_wheel_name  = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.rear_left_wheel_name   = info_.hardware_parameters["rear_left_wheel_name"];
  cfg_.rear_right_wheel_name  = info_.hardware_parameters["rear_right_wheel_name"];

  cfg_.loop_rate  = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device     = info_.hardware_parameters["device"];
  cfg_.baud_rate  = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout    = std::stoi(info_.hardware_parameters["timeout"]);

  cfg_.enc_counts_per_rev_fl = std::stoi(info_.hardware_parameters["enc_counts_per_rev_fl"]);
  cfg_.enc_counts_per_rev_fr = std::stoi(info_.hardware_parameters["enc_counts_per_rev_fr"]);
  cfg_.enc_counts_per_rev_rl = std::stoi(info_.hardware_parameters["enc_counts_per_rev_rl"]);
  cfg_.enc_counts_per_rev_rr = std::stoi(info_.hardware_parameters["enc_counts_per_rev_rr"]);

  /* Setup wheel objects.
     FL and RL shared the average encoder (from Arduino), so
     they are configured by the average CPR of lrft side.
     And vice versa */
  fl_wheel_.setup(cfg_.front_left_wheel_name,  cfg_.enc_counts_per_rev_left());
  rl_wheel_.setup(cfg_.rear_left_wheel_name,   cfg_.enc_counts_per_rev_left());
  fr_wheel_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev_right());
  rr_wheel_.setup(cfg_.rear_right_wheel_name,  cfg_.enc_counts_per_rev_right());

  // Set up the Arduino
  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

  RCLCPP_INFO(logger_, "Finished Configuration");
  
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  /* Export velocity and position state for all 4 wheels */
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

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  /* Export velocity command interface for all 4 wheels.
     diff_drive_controller will write on the FL/RL (left) and the FR/RR (right). */
  command_interfaces.emplace_back(fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.cmd);
  command_interfaces.emplace_back(fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.cmd);
  command_interfaces.emplace_back(rl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rl_wheel_.cmd);
  command_interfaces.emplace_back(rr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rr_wheel_.cmd);

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveArduino::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Starting 4WD Controller...");
  arduino_.sendEmptyMsg();
  // arduino.setPidValues(9,7,0,100);
  // arduino.setPidValues(14,7,0,100);
  arduino_.setPidValues(30, 20, 0, 100);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduino::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Stopping 4WD Controller...");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduino::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO fix chrono duration

  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;

  if (!arduino_.connected())
    return return_type::ERROR;

  /* Arduino sends two value: avg_left and avg_right
     This value is asigned to ALL wheels on the same side
     so the odometry controller gets a consistent state */
  int avg_left, avg_right;
  arduino_.readEncoderValues(avg_left, avg_right);

  /* Update encoder count -> FL and RL get the same value */
  fl_wheel_.enc = avg_left;
  rl_wheel_.enc = avg_left;

  fr_wheel_.enc = avg_right;
  rr_wheel_.enc = avg_right;

  /* Calculate the position and the velocity for all 4 wheels */
  auto updateWheel = [&](Wheel & w, double & pos_prev_ref) {
    double pos_prev = pos_prev_ref;
    w.pos = w.calcEncAngle();
    w.vel = (w.pos - pos_prev) / deltaSeconds;
    pos_prev_ref = w.pos;
  };

  double tmp;
  tmp = fl_wheel_.pos; updateWheel(fl_wheel_, tmp);
  tmp = fr_wheel_.pos; updateWheel(fr_wheel_, tmp);
  tmp = rl_wheel_.pos; updateWheel(rl_wheel_, tmp);
  tmp = rr_wheel_.pos; updateWheel(rr_wheel_, tmp);

  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!arduino_.connected())
    return return_type::ERROR;

  /* Taking command per side, diff_drive_controller writes in FL and RL
     with the same value as (left), then FR anf RR with the same value as (right)
     The average FL+RL and FR+RR for robustness if the controller
     gives the little bit of different value */
  double left_cmd  = (fl_wheel_.cmd + rl_wheel_.cmd) / 2.0;
  double right_cmd = (fr_wheel_.cmd + rr_wheel_.cmd) / 2.0;

  /* Convers rad/s to ticks/frame, similiar to 2WD version */
  int left_ticks  = static_cast<int>(left_cmd  / fl_wheel_.rads_per_count / cfg_.loop_rate);
  int right_ticks = static_cast<int>(right_cmd / fr_wheel_.rads_per_count / cfg_.loop_rate);

  arduino_.setMotorValues(left_ticks, right_ticks);

  return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino, 
  hardware_interface::SystemInterface
)
