#include <wiringPi.h>
#include "ros/ros.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <thread>
#include <signal.h>

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

  ros::NodeHandle nh_;

  ros::Rate rate_;

public:
  platform_run();
  void pins_setup();

  void odometry();
  void safety_task();

  void move(bool forw);

};

#endif

