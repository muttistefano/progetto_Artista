#include<pkg_movimento/movimento_node.h>
#include<pkg_movimento/pins_def.h>




platform_run::platform_run()
{

  this->nh_ = ros::NodeHandle nh_("~");

  this->pins_setup();

  if (this->log_to_file_)
  {
    
    this->logfile_.open ("log.csv");
  }

  if (this->nh_.getParam("movement_length_", this->movement_length_))
  {
    ROS_INFO("Got param movement_length");
  }

  this->sub_odom_ = this->nh_.subscribe<std_msgs::Float64MultiArray>("odom_error", 1, [&] (const auto &msg) {this->odom_error_ = (*msg).data; });
  std::thread t1(&platform_run::safety_task, this);
  t1.detach();

  std::thread t2(&platform_run::odometry, this);
  t2.detach();

}

void platform_run::safety_task()
{
  //TODO interrupr?
  while (ros::ok())
  {
     this->safe_ = digitalRead(bumper_pin);
     this->rate_.sleep();
  }
}

void my_handler(int s){
  printf("Caught signal %d\n",s);
  softPwmWrite(PWM_pin_1,0);
  softPwmWrite(PWM_pin_2,0);
  softPwmStop   (PWM_pin_1) ;
  softPwmStop   (PWM_pin_2) ;
  exit(1); 

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
  digitalWrite(PWM_dir_1,HIGH);
  digitalWrite(PWM_dir_2,HIGH);

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
}

void platform_run::odometry()
{
  // integrare angolo
  
  while(ros::ok())
  {
    this->enc_diff_   = (posR1_ + posR2_) - (posL1_ + posL2_);
    this->curr_pos_   = (posR1_ + posR2_ + posL1_ + posL2_)/4.0;
    this->corr_left_  =        VIS_GAIN * (this->rho_ref - this->odom_error_[0]) + ENC_GAIN * enc_diff_;
    this->corr_right_ = -1.0 * VIS_GAIN * (this->rho_ref - this->odom_error_[0]) - ENC_GAIN * enc_diff_;
    this->rate_.sleep();
  }
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
  
  int corr      = forw ? 1:-1;
  int intensity = 0;
	

  while(this->curr_pos_ < (0.2 * this->movement_length_))
  {
    softPwmWrite (PWM_pin_1, (intensity + (corr * int(this->corr_left_))  * this->safe_)) ;	
    softPwmWrite (PWM_pin_2, (intensity + (corr * int(this->corr_right_)) * this->safe_)) ;	
    ROS_DEBUG_STREAM_THROTTLE(0.5,"init" << (intensity + (corr * int(this->corr_left_)) * this->safe_) << " " << (intensity + (corr * int(this->corr_right_)) * this->safe_));
    intensity = (intensity < (MAX_PWM_RANGE/2)) ? (intensity + 1) : intensity;
    if(this->log_to_file_)
    {
      this->logfile_ << "init," << (intensity + (corr * int(this->corr_left_)) * this->safe_) << "," << (intensity + (corr * int(this->corr_right_)) * this->safe_) << ","
                     << this->odom_error_[0] << "," << this->odom_error_[1] << "," << this->enc_diff_ <<  "," << this->curr_pos_ << "\n";
    }
    delay (acc_delay) ;
  }
  
  while(this->curr_pos_ < (0.8 * this->movement_length_))
  {
    softPwmWrite (PWM_pin_1, (intensity + (corr * int(this->corr_left_)) * this->safe_)) ;	
    softPwmWrite (PWM_pin_2, (intensity + (corr * int(this->corr_right_)) * this->safe_)) ;	
    ROS_DEBUG_STREAM_THROTTLE(0.5,"Stall" << (intensity + (corr * int(this->corr_left_)) * this->safe_) << " " << (intensity + (corr * int(this->corr_right_)) * this->safe_));
    if(this->log_to_file_)
    {
      this->logfile_ << "init," << (intensity + (corr * int(this->corr_left_)) * this->safe_) << "," << (intensity + (corr * int(this->corr_right_)) * this->safe_) << ","
                     << this->odom_error_[0] << "," << this->odom_error_[1] << "," << this->enc_diff_ <<  "," << this->curr_pos_ << "\n";
    }
    delay (10) ;
  }

  while(this->curr_pos_ < this->movement_length_)
  {
    softPwmWrite (PWM_pin_1, (intensity + (corr * int(this->corr_right_)) * this->safe_)) ;	
    softPwmWrite (PWM_pin_2, (intensity + (corr * int(this->corr_left_)) * this->safe_)) ;	
    ROS_DEBUG_STREAM_THROTTLE(0.5,"final" << (intensity + (corr * int(this->corr_left_)) * this->safe_) << " " << (intensity + (corr * int(this->corr_right_)) * this->safe_));
    intensity = (intensity > (MAX_PWM_RANGE/4)) ? (intensity - 1) : intensity;
    if(this->log_to_file_)
    {
      this->logfile_ << "init," << (intensity + (corr * int(this->corr_left_)) * this->safe_) << "," << (intensity + (corr * int(this->corr_right_)) * this->safe_) << ","
                     << this->odom_error_[0] << "," << this->odom_error_[1] << "," << this->enc_diff_ <<  "," << this->curr_pos_ << "\n";
    }
    delay (acc_delay) ;
  }
  
  delay(5000);

}

int main(int argc, char **argv)
{      

  struct sched_param param;
          pthread_attr_t attr;
          pthread_t thread;
          int ret;
  
  /* Lock memory */
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
          printf("mlockall failed: %m\n");
          exit(-2);
  }

  /* Initialize pthread attributes (default values) */
  ret = pthread_attr_init(&attr);
  if (ret) {
          printf("init pthread attributes failed\n");
          exit(-2);
  }

  /* Set a specific stack size  */
  ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
  if (ret) {
      printf("pthread setstacksize failed\n");
      exit(-2);
  }

  /* Set scheduler policy and priority of pthread */
  ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (ret) {
          printf("pthread setschedpolicy failed\n");
        exit(-2);
  }

  param.sched_priority = 80;
  ret = pthread_attr_setschedparam(&attr, &param);
  if (ret) {
          printf("pthread setschedparam failed\n");
          exit(-2);
  }
  /* Use scheduling parameters of attr */
  ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (ret) {
          printf("pthread setinheritsched failed\n");
          exit(-2);
  }

  if (wiringPiSetup () == -1)
    exit (1) ;

  ros::init(argc, argv, "run_plat");

  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);

  platform_run plat = platform_run();


  ros::AsyncSpinner spinner(4); 
  spinner.start();

  while(ros::ok())
  {
    plat.move(true);
    plat.move(false);
  }


  // plat.move(true);

  ros::waitForShutdown();


}
