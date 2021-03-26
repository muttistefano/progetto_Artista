#include<pkg_movimento/movimento_node.h>
#include<pkg_movimento/pins_def.h>

platform_run::platform_run():rate_(ros::Rate(50))
{

  this->pins_setup();
  this->sub_odom_ = this->nh_.subscribe<std_msgs::Float64MultiArray>("odom_error", 1, [&] (const auto &msg) {this->odom_error_ = (*msg).data; });
  std::thread t1(&platform_run::safety_task, this);
  t1.detach();

  std::thread t2(&platform_run::odometry, this);
  t2.detach();

}

void platform_run::safety_task()
{

  while (ros::ok())
  {
     this->safe_ = digitalRead(bumper_pin);
     this->rate_.sleep();
  }
}

void my_handler(int s){
  printf("Caught signal %d\n",s);
  pwmWrite (PWM_pin_1, 0) ;	
  pwmWrite (PWM_pin_2, 0) ;	
  exit(1); 

}

void platform_run::pins_setup()
{
  
  pinMode (PWM_pin_1, PWM_OUTPUT) ; 
  pinMode (PWM_pin_2, PWM_OUTPUT) ; 
  pinMode (PWM_dir_1, OUTPUT) ;
  pinMode (PWM_dir_2, OUTPUT) ;

  pinMode (bumper_pin, INPUT) ;
  pullUpDnControl(bumper_pin,PUD_UP);

  //TODO in base a dove sei
  digitalWrite(PWM_dir_1,HIGH);
  digitalWrite(PWM_dir_2,HIGH);
  ROS_DEBUG("Pin setup finished");

}

void platform_run::odometry()
{
  while(ros::ok())
  {
    this->corr_left_  =        (this->rho_ref - this->odom_error_[0]);
    this->corr_right_ = -1.0 * (this->rho_ref - this->odom_error_[0]);
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
  
  int corr = forw ? 1:-1;
  // while ((this->curr_pos_ < this->open_dist_) && ros::ok())
  // {

  //   this->pwm_current_ = (  abs(this->curr_pos_ - this->open_dist_) < this->ramp_lenght_  ) ? (abs(this->curr_pos_ - this->open_dist_)/this->ramp_lenght_) * this->pwm_max_ : this->pwm_max_;

  //   pwmWrite (PWM_pin_1, this->pwm_current_ * this->safe_ + int(this->corr_left_)) ;	
  //   pwmWrite (PWM_pin_2, this->pwm_current_ * this->safe_ + int(this->corr_right_)) ;	
  //   // std::cout << this->pwm_current_ <<  this->safe_ << int(this->corr_left_) << int(this->corr_right_) << "  " << forw <<  std::endl;
  //   ROS_DEBUG_STREAM_THROTTLE(2,this->pwm_current_ << "  " <<  this->safe_ << "  " << int(this->corr_left_) << "  " << int(this->corr_right_) << "  " << forw);
  //   this->rate_.sleep() ;
    
  // }	


    for (int intensity = 200 ; intensity < 800 ; intensity++)
    {
      pwmWrite (PWM_pin_1, (intensity + (corr * int(this->corr_left_))  * this->safe_)) ;	
      pwmWrite (PWM_pin_2, (intensity + (corr * int(this->corr_right_)) * this->safe_)) ;	
      ROS_DEBUG_STREAM_THROTTLE(0.5,"init" << (intensity + (corr * int(this->corr_left_)) * this->safe_) << " " << (intensity + (corr * int(this->corr_right_)) * this->safe_));
      delay (3) ;
    }
    
    for (int tm = 0 ; tm <= 2000 ; tm++)
    {
      pwmWrite (PWM_pin_1, (800 + (corr * int(this->corr_left_)) * this->safe_)) ;	
      pwmWrite (PWM_pin_2, (800 + (corr * int(this->corr_right_)) * this->safe_)) ;	
      ROS_DEBUG_STREAM_THROTTLE(0.5,"Stall" << (800 + (corr * int(this->corr_left_)) * this->safe_) << " " << (800 + (corr * int(this->corr_right_)) * this->safe_));
      delay (10) ;
    }

    for (int intensity = 800 ; intensity >= 200 ; intensity--)
    {
      pwmWrite (PWM_pin_1, (intensity + (corr * int(this->corr_right_)) * this->safe_)) ;	
      pwmWrite (PWM_pin_2, (intensity + (corr * int(this->corr_left_)) * this->safe_)) ;	
      ROS_DEBUG_STREAM_THROTTLE(0.5,"final" << (intensity + (corr * int(this->corr_left_)) * this->safe_) << " " << (intensity + (corr * int(this->corr_right_)) * this->safe_));
      delay (3) ;
    }
    delay(5000);

}

int main(int argc, char **argv)
{      

  if (wiringPiSetup () == -1)
    exit (1) ;

  ros::init(argc, argv, "run_plat");

  // struct sigaction sigIntHandler;

  // sigIntHandler.sa_handler = my_handler;
  // sigemptyset(&sigIntHandler.sa_mask);
  // sigIntHandler.sa_flags = 0;

  // sigaction(SIGINT, &sigIntHandler, NULL);

  platform_run plat = platform_run();

  ros::AsyncSpinner spinner(4); 
  spinner.start();

  // while(ros::ok())
  // {
  //   plat.move(true);
  //   plat.move(false);
  // }


  plat.move(true);

  ros::waitForShutdown();

  printf("Caught signal\n");
  pwmWrite (PWM_pin_1, 0) ;	
  pwmWrite (PWM_pin_2, 0) ;	

}
