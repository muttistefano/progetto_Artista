#include <wiringPi.h>
#include "ros/ros.h"
#include <ros/console.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <thread>
#include <signal.h>

#include <std_msgs/Float64MultiArray.h>

#ifndef MOVIMENTO_HH
#define MOVIMENTO_HH

class platform_run
{

  // pos data
  double open_dist_;
  double curr_pos_;

  // speed data
  int pwm_max_;
  int pwm_current_;
  int ramp_delay_ms_ ;
  int ramp_lenght_;

  // safety data
  int safe_;

  // correction data
  std::vector<double> odom_error_{  276.8,  276.8 };
  double rho_ref   = 276.8;
  double theta_ref = 0.0383972;
  double corr_right_;
  double corr_left_;

  ros::NodeHandle nh_;
  ros::Rate rate_;

  ros::Subscriber sub_odom_;

public:
  platform_run();
  void pins_setup();

  void odometry();
  void safety_task();

  void move(bool forw);

};

#endif

