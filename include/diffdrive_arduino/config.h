#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>

struct Config
{
  /* Wheels' name fot the ros2_control */
  std::string front_left_wheel_name  = "front_left_wheel";
  std::string front_right_wheel_name = "front_right_wheel";
  std::string rear_left_wheel_name   = "rear_left_wheel";
  std::string rear_right_wheel_name  = "rear_right_wheel";

  float loop_rate = 30;

  std::string device   = "/dev/ttyUSB0";
  int baud_rate        = 57600;
  int timeout          = 1000;

  /* Encoder counts per revolution - each wheel can be different,
     if the encoder resolutions are not perfectly match.
     Arduino sends tha average per side, so this value must be
     consist of the encoder that installed on each wheel.
     Default: 1493 CPR (this value is the exact motor I used for 4WD AGV). */
  int enc_counts_per_rev_fl = 1493;
  int enc_counts_per_rev_fr = 1493;
  int enc_counts_per_rev_rl = 1493;
  int enc_counts_per_rev_rr = 1493;

  /* The encoder counts per rev repost to the ROS2.
     Because, Arduino sends the average (FL+RL)/2 and (FR+RR)/2,
     These values are the average from pair of FL+RL and FR+RR.
     If all the encoder have the same value, this will be equal to enc_counts_per_rev_fl. */
  int enc_counts_per_rev_left() const {
    return (enc_counts_per_rev_fl + enc_counts_per_rev_rl) / 2;
  }
  int enc_counts_per_rev_right() const {
    return (enc_counts_per_rev_fr + enc_counts_per_rev_rr) / 2;
  }
};

#endif // DIFFDRIVE_ARDUINO_CONFIG_H
