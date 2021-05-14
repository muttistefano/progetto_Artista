#include<pkg_movimento/movimento_node.h>
#include<pkg_movimento/motion_config.h>
#include <signal.h>


void softPwmWrite_th(int pin,int value)
{
  int value_th = (value > 3)  ? value : 3;
      value_th = (value_th < MAX_PWM_RANGE) ? value_th : MAX_PWM_RANGE;
  softPwmWrite(pin, value_th);
}

platform_run::platform_run(ros::NodeHandle nh_)
{


  this->pins_setup();

  if (this->log_to_file_)
  {
    
    this->logfile_.open ("log.csv");
  }

  if (nh_.getParam("movement_length", this->movement_length_))
  {
    ROS_INFO("Got param movement_length");
  }
  else
  {
    ROS_FATAL("No movement length param");
  }

  if (nh_.getParam("rho_ref", this->rho_ref))
  {
    ROS_INFO("Got param rho_ref");
  }
  else
  {
    ROS_FATAL("No rho_ref  param");
  }

  if (nh_.getParam("theta_ref", this->theta_ref))
  {
    ROS_INFO("Got param theta_ref");
  }
  else
  {
    ROS_FATAL("No theta_ref  param");
  }

  this->sub_odom_ = nh_.subscribe<std_msgs::Float64MultiArray>("odom_error", 1, [&] (const auto &msg) {this->odom_error_ = (*msg).data; });
  std::thread t1(&platform_run::safety_task, this);
  t1.detach();

  pid_pos_ = new PID(0.01,0.001,0.0,2.0); 

  ROS_INFO("PID pos st point %f",this->rho_ref);
  pid_pos_->set_set_point(this->rho_ref);

  std::thread t2(&platform_run::odometry, this);
  t2.detach();

}

void platform_run::safety_task()
{
  //TODO interrupr?
  while (ros::ok())
  {
     this->safe_ = 1;//digitalRead(bumper_pin);
     this->rate_.sleep();
  }
}

void my_handler(int s){
  printf("Caught signal %d\n",s);
  softPwmWrite(PWM_pin_1,0);
  softPwmWrite(PWM_pin_2,0);
  softPwmStop   (PWM_pin_1) ;
  softPwmStop   (PWM_pin_2) ;
  ros::shutdown();

}

void platform_run::pins_setup()
{
  
  softPwmCreate (PWM_pin_1, 0, MAX_PWM_RANGE) ;
  softPwmCreate (PWM_pin_2, 0, MAX_PWM_RANGE) ;
  pinMode (PWM_dir_1, OUTPUT) ;
  pinMode (PWM_dir_2, OUTPUT) ;

  pinMode (bumper_pin, INPUT) ;
  pullUpDnControl(bumper_pin,PUD_UP);

  //TODO in base a dove sei
  // digitalWrite(PWM_dir_1,HIGH);
  // digitalWrite(PWM_dir_2,HIGH);

  pinMode (EncR1,INPUT);
  pinMode (EncR2,INPUT);
  pinMode (EncL1,INPUT);
  pinMode (EncL2,INPUT);

  if (wiringPiISR (EncR1, INT_EDGE_RISING, &interruptR1) < 0)
  {
    ROS_DEBUG("Interrupt setup error");
    return ;
  }

  if (wiringPiISR (EncR2, INT_EDGE_RISING, &interruptR2) < 0)
  {
    ROS_DEBUG("Interrupt setup error"); 
  }

  if (wiringPiISR (EncL1, INT_EDGE_RISING, &interruptL1) < 0)
  {
    ROS_DEBUG("Interrupt setup error");
    return ;
  }

  if (wiringPiISR (EncL2, INT_EDGE_RISING, &interruptL2) < 0)
  {
    ROS_DEBUG("Interrupt setup error");
    return ;
  }

  ROS_DEBUG("Pin setup finished");

}

void platform_run::reset_odom()
{
  Counter1_ = 0;
  Counter2_ = 0;
  Counter3_ = 0;
  Counter4_ = 0;
  posR1_    = 0.0;
  posR2_    = 0.0;
  posL1_    = 0.0;
  posL2_    = 0.0;
  this->curr_pos_ = 0.0; 
}

void platform_run::odometry()
{
  // integrare angolo
  
  while(ros::ok())
  {
    double THETA_GAIN = 10;
    double correction = this->pid_pos_->output( this->odom_error_[0]) - THETA_GAIN * (this->theta_ref - this->odom_error_[1]) ;
    this->enc_diff_   = (posR1_ + posR2_) - (posL1_ + posL2_);
    this->curr_pos_   = (posR1_ + posR2_ + posL1_ + posL2_)/4.0;
    this->corr_left_  =         correction ;
    this->corr_right_ =  -1.0 * correction ;
    // std::cout << this->corr_left_ << " " << this->corr_right_ << "\n";
    std::cout << "VIS : " <<THETA_GAIN * (this->theta_ref - this->odom_error_[1]) << " " << this->theta_ref << " " << this->odom_error_[1] << "\n";
    this->rate_.sleep();
  }
}

int  platform_run::get_nom_speed()
{
  int vel_ret = 0;

  if(this->curr_pos_ < (perc_ramp * this->movement_length_))
  {
    vel_ret = int( this->curr_pos_ * (max_speed/(perc_ramp * this->movement_length_))) + MIN_VEL_TH;
    return vel_ret;
  }
  if(this->curr_pos_ > (this->movement_length_ - (perc_ramp * this->movement_length_)))
  {
    vel_ret = int( (this->movement_length_ - this->curr_pos_) * (max_speed/(perc_ramp * this->movement_length_))) + MIN_VEL_TH;
    return vel_ret;
  }
  if(this->curr_pos_ < this->movement_length_ )
  {
    return int(max_speed);
  }
  std::cout << "EEERRRROOORORORORORROROROOR";
}

void platform_run::move(bool forw)
{
  
  if (forw)
  {
    digitalWrite(PWM_dir_1,HIGH);
    digitalWrite(PWM_dir_2,HIGH);
  }else
  {
    digitalWrite(PWM_dir_1,LOW);
    digitalWrite(PWM_dir_2,LOW);
  }
  
  int corr      = 1;//forw ? 1:-1;
  int intensity = 0;
	
  this->pid_pos_->reset();
  
  // while(this->curr_pos_ < (0.2 * this->movement_length_))
  // {
  //   softPwmWrite_th (PWM_pin_1, (intensity + (corr * int(this->corr_left_)))  * this->safe_) ;	
  //   softPwmWrite_th (PWM_pin_2, (intensity + (corr * int(this->corr_right_))) * this->safe_) ;	
  //   ROS_DEBUG_STREAM_THROTTLE(0.2,"init," << (intensity + (corr * int(this->corr_left_))) * this->safe_ << "," << (intensity + (corr * int(this->corr_right_))) * this->safe_ << ","
  //                                         << this->odom_error_[0] << "," << this->odom_error_[1] << "," << this->enc_diff_ <<  "," << this->curr_pos_ << "," << corr * int(this->corr_left_) << "," << corr * int(this->corr_right_));
  //   intensity = (intensity < (MAX_PWM_RANGE/3)) ? (intensity + 1) : intensity;
  //   if(this->log_to_file_)
  //   {
  //     this->logfile_ << "init," << (intensity + (corr * int(this->corr_left_))) * this->safe_ << "," << (intensity + (corr * int(this->corr_right_))) * this->safe_ << ","
  //                    << this->odom_error_[0] << "," << this->odom_error_[1] << "," << this->enc_diff_ <<  "," << this->curr_pos_ << "," << corr * int(this->corr_left_) << "," << corr * int(this->corr_right_) << "\n";
  //   }
  //   delay (acc_delay) ;
  // }
  
  // while(this->curr_pos_ < (0.8 * this->movement_length_))
  // {
  //   softPwmWrite_th (PWM_pin_1, (intensity + (corr * int(this->corr_left_))) * this->safe_) ;	
  //   softPwmWrite_th (PWM_pin_2, (intensity + (corr * int(this->corr_right_))) * this->safe_) ;	
  //   ROS_DEBUG_STREAM_THROTTLE(0.2,"stall," << (intensity + (corr * int(this->corr_left_))) * this->safe_ << "," << (intensity + (corr * int(this->corr_right_)) )* this->safe_ << ","
  //                    << this->odom_error_[0] << "," << this->odom_error_[1] << "," << this->enc_diff_ <<  "," << this->curr_pos_ << "," << corr * int(this->corr_left_) << "," << corr * int(this->corr_right_));
  //   if(this->log_to_file_)
  //   {
  //     this->logfile_ << "stall," << (intensity + (corr * int(this->corr_left_))) * this->safe_ << "," << (intensity + (corr * int(this->corr_right_)) )* this->safe_ << ","
  //                    << this->odom_error_[0] << "," << this->odom_error_[1] << "," << this->enc_diff_ <<  "," << this->curr_pos_ << "," << corr * int(this->corr_left_) << "," << corr * int(this->corr_right_) << "\n";
  //   }
  //   delay (10) ;
  // }

  // while(this->curr_pos_ < this->movement_length_)
  // {
  //   softPwmWrite_th (PWM_pin_1, (intensity + (corr * int(this->corr_left_))) * this->safe_) ;	
  //   softPwmWrite_th (PWM_pin_2, (intensity + (corr * int(this->corr_right_))) * this->safe_) ;	
  //   ROS_DEBUG_STREAM_THROTTLE(0.2,"final," << (intensity + (corr * int(this->corr_left_))) * this->safe_ << "," << (intensity + (corr * int(this->corr_right_))) * this->safe_ << ","
  //                    << this->odom_error_[0] << "," << this->odom_error_[1] << "," << this->enc_diff_ <<  "," << this->curr_pos_ << "," << corr * int(this->corr_left_) << "," << corr * int(this->corr_right_));
  //   intensity = (intensity > (MAX_PWM_RANGE/12)) ? (intensity - 1) : intensity;
  //   if(this->log_to_file_)
  //   {
  //     this->logfile_ << "final," << (intensity + (corr * int(this->corr_left_))) * this->safe_ << "," << (intensity + (corr * int(this->corr_right_))) * this->safe_ << ","
  //                    << this->odom_error_[0] << "," << this->odom_error_[1] << "," << this->enc_diff_ <<  "," << this->curr_pos_ << "," << corr * int(this->corr_left_) << "," << corr * int(this->corr_right_) << "\n";
  //   }
  //   delay (acc_delay) ;
  // }
  
  while(this->curr_pos_ < this->movement_length_)
  {
    intensity = get_nom_speed();
    softPwmWrite_th (PWM_pin_1, (intensity + (corr * int(this->corr_left_)))  * this->safe_) ;	
    softPwmWrite_th (PWM_pin_2, (intensity + (corr * int(this->corr_right_))) * this->safe_) ;	

    std::string deb_str = std::to_string((intensity + (corr * int(this->corr_left_))) * this->safe_)  + std::string(",")
                        + std::to_string((intensity + (corr * int(this->corr_right_))) * this->safe_) + std::string(",")
                        + std::to_string(this->odom_error_[0]) + std::string(",") + std::to_string(this->odom_error_[1]) + std::string(",")
                        + std::to_string(this->enc_diff_) + std::string(",") + std::to_string(this->curr_pos_ ) + std::string(",")
                        + std::to_string(corr * int(this->corr_left_)) + std::string(",") + std::to_string(corr * int(this->corr_right_));

    ROS_DEBUG_STREAM_THROTTLE(0.1,deb_str);

    if(this->log_to_file_){this->logfile_ << deb_str << "\n";}
    delay (acc_delay) ;
  }
  softPwmWrite (PWM_pin_1, 0) ;	
  softPwmWrite (PWM_pin_2, 0) ;	
  delay(3000);

}

int main(int argc, char **argv)
{      

  // struct sched_param param;
  //         pthread_attr_t attr;
  //         pthread_t thread;
  //         int ret;
  
  // /* Lock memory */
  // if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
  //         printf("mlockall failed: %m\n");
  //         exit(-2);
  // }

  // /* Initialize pthread attributes (default values) */
  // ret = pthread_attr_init(&attr);
  // if (ret) {
  //         printf("init pthread attributes failed\n");
  //         exit(-2);
  // }

  // /* Set a specific stack size  */
  // ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
  // if (ret) {
  //     printf("pthread setstacksize failed\n");
  //     exit(-2);
  // }

  // /* Set scheduler policy and priority of pthread */
  // ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  // if (ret) {
  //         printf("pthread setschedpolicy failed\n");
  //       exit(-2);
  // }

  // param.sched_priority = 80;
  // ret = pthread_attr_setschedparam(&attr, &param);
  // if (ret) {
  //         printf("pthread setschedparam failed\n");
  //         exit(-2);
  // }
  // /* Use scheduling parameters of attr */
  // ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  // if (ret) {
  //         printf("pthread setinheritsched failed\n");
  //         exit(-2);
  // }

  if (wiringPiSetup () == -1)
    exit (1) ;

  ros::init(argc, argv, "run_plat", ros::init_options::NoSigintHandler);

  struct sigaction sigIntHandler;

  // sigIntHandler.sa_handler = my_handler;
  // sigemptyset(&sigIntHandler.sa_mask);
  // sigIntHandler.sa_flags = 0;

  // sigaction(SIGINT, &sigIntHandler, NULL);
  signal(SIGINT, my_handler);

  ros::NodeHandle nh;
  platform_run plat = platform_run(nh);


  ros::AsyncSpinner spinner(4); 
  spinner.start();
  sleep(2);

  while(ros::ok())
  {
    plat.reset_odom();
    std::cout << "moving forward\n";
    plat.reset_odom();
    plat.move(true);
    std::cout << "reset odom\n";
    sleep(2);
    std::cout << "moving backward\n";
    plat.reset_odom();
    plat.move(false);
    std::cout << "reset odom\n";
    sleep(2);
  }


  // plat.move(true);

  ros::waitForShutdown();


}
