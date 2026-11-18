#include "diffdrive_arduino/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>
#include <thread>
#include <chrono>

/* arduino_comms.cpp - there is no logic difference from 2WD version.
   The readEncoderValues() function still recieves the 2 integer values,
   which now is representing the AVERAGE per side (FL+RL)/2 and (FR+RR)/2.
   The average calculation is done in the Arduino side (diff_controller.h),
   it is not here. */

void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
  serial_conn_.setPort(serial_device);
  serial_conn_.setBaudrate(baud_rate);
  serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
  serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
  serial_conn_.open();
  // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));
  
  // Mega auto-resets when the port opens: wait for boot, then drop any noise
  std::this_thread::sleep_for(std::chrono::seconds(2));
  serial_conn_.flushInput();
}

void ArduinoComms::sendEmptyMsg()
{
  std::string response = sendMsg("\r");
}

void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
  /* Replying from Arduino: "avg_left avg_right\r\n"
     val_1 = average encoder left side  (FL + RL) / 2
     val_2 = average encoder right side (FR + RR) / 2 */
  std::string response = sendMsg("e\r");

  std::string delimiter = " ";
  size_t del_pos = response.find(delimiter);
  std::string token_1 = response.substr(0, del_pos);
  std::string token_2 = response.substr(del_pos + delimiter.length());

  val_1 = std::atoi(token_1.c_str());
  val_2 = std::atoi(token_2.c_str());
}

void ArduinoComms::setMotorValues(int val_1, int val_2)
{
  /* val_1 = ticks/frame left side  -> Arduino broadcasts to FL and RL
     val_2 = ticks/frame right side -> Arduino broadcasts to FR and RR */
  std::stringstream ss;
  ss << "m " << val_1 << " " << val_2 << "\r";
  sendMsg(ss.str(), false);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
  /* The same PID is applied to all of the 4 motors in Arduino */
  std::stringstream ss;
  ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
  sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
  serial_conn_.write(msg_to_send);
  std::string response = serial_conn_.readline();

  if (print_output)
  {
      // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
      // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
  }
  
  return response;
}
