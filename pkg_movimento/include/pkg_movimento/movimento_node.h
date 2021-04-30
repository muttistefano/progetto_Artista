#include <wiringPi.h>
#include "ros/ros.h"
#include <ros/console.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <thread>
#include <signal.h>
#include <chrono>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <std_msgs/Float64MultiArray.h>

#ifndef MOVIMENTO_HH
#define MOVIMENTO_HH

std::chrono::time_point<std::chrono::system_clock> timeR1 = std::chrono::high_resolution_clock::now();
std::chrono::time_point<std::chrono::system_clock> timeR2 = std::chrono::high_resolution_clock::now();
std::chrono::time_point<std::chrono::system_clock> timeL1 = std::chrono::high_resolution_clock::now();
std::chrono::time_point<std::chrono::system_clock> timeL2 = std::chrono::high_resolution_clock::now();

double velR1 = 0.0;
double velR2 = 0.0;
double velL1 = 0.0;
double velL2 = 0.0;

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
  ros::Rate rate_ = ros::Rate(50);

  ros::Subscriber sub_odom_;

public:
  platform_run();
  void pins_setup();

  void odometry();
  void safety_task();

  void move(bool forw);

  // static void interruptR1();
  // static void interruptR2();
  // static void interruptL1();
  // static void interruptL2();

};

#endif

