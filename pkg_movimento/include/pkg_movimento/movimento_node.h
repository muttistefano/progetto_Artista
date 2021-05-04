#include <wiringPi.h>
#include <softPwm.h>
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
#include <fstream>


#ifndef MOVIMENTO_HH
#define MOVIMENTO_HH

//Encoders data
double posR1_ = 0.0;
double posR2_ = 0.0;
double posL1_ = 0.0;
double posL2_ = 0.0;

double Counter1_ = 0 ;
double Counter2_ = 0 ;
double Counter3_ = 0 ;
double Counter4_ = 0 ;

class platform_run
{

  //ros
  ros::NodeHandle nh_("~");

  //logging
  bool log_to_file_ = true;
  std::ofstream logfile_;

  // pos data
  double movement_length_;
  double curr_pos_;
  double enc_diff_ = 0.0;


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
  void reset_odom();

};

void interruptR1()
{
  ++Counter1_ ;
  posR1_ = (Counter1_/520.0) * 2.0 * M_PI * 0.06;
}

void interruptR2()
{
  ++Counter2_ ;
  posR2_ = (Counter2_/520.0) * 2.0 * M_PI * 0.06;
}

void interruptL1()
{
  ++Counter3_ ;
  posL1_ = (Counter3_/520.0) * 2.0 * M_PI * 0.06;
}

void interruptL2()
{
  ++Counter4_ ;
  posL2_ = (Counter4_/520.0) * 2.0 * M_PI * 0.06;
}

#endif

