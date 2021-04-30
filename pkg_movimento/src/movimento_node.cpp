#include<pkg_movimento/movimento_node.h>
#include<pkg_movimento/pins_def.h>


void interruptR1()
{
  auto start = std::chrono::high_resolution_clock::now();
  double elaps =  std::chrono::duration<double>(timeR1 - start).count();
  velR1  = 1.0/elaps;
  timeR1 = start;
}

void interruptR2()
{
  auto start = std::chrono::high_resolution_clock::now();
  double elaps =  std::chrono::duration<double>(timeR2 - start).count();
  velR2  = 1.0/elaps;
  timeR2 = start;
}

void interruptL1()
{
  auto start = std::chrono::high_resolution_clock::now();
  double elaps =  std::chrono::duration<double>(timeL1 - start).count();
  velL1  = 1.0/elaps;
  timeL1 = start;
}

void interruptL2()
{
  auto start = std::chrono::high_resolution_clock::now();
  double elaps =  std::chrono::duration<double>(timeL2 - start).count();
  velL2  = 1.0/elaps;
  timeL2 = start;
}


platform_run::platform_run()
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

// void platform_run::interruptR1()
// {
//   auto start = std::chrono::high_resolution_clock::now();
//   double elaps =  std::chrono::duration_cast<std::chrono::seconds>(timeR1 - start).count();
//   velR1  = 1.0/elaps;
//   timeR1 = start;
// }

// void platform_run::interruptR2()
// {
//   auto start = std::chrono::high_resolution_clock::now();
//   double elaps =  std::chrono::duration_cast<std::chrono::seconds>(timeR2 - start).count();
//   velR2  = 1.0/elaps;
//   timeR2 = start;
// }

// void platform_run::interruptL1()
// {
//   auto start = std::chrono::high_resolution_clock::now();
//   double elaps =  std::chrono::duration_cast<std::chrono::seconds>(timeL1 - start).count();
//   velL1  = 1.0/elaps;
//   timeL1 = start;
// }

// void platform_run::interruptL2()
// {
//   auto start = std::chrono::high_resolution_clock::now();
//   double elaps =  std::chrono::duration_cast<std::chrono::seconds>(timeL2 - start).count();
//   std::cout << elaps << "\n";
//   velL2  = 1.0/elaps;
//   timeL2 = start;
// }

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

  pinMode (EncR1,INPUT);
  pinMode (EncR2,INPUT);
  pullUpDnControl(EncR1,PUD_UP);
  pullUpDnControl(EncR2,PUD_UP);
  pinMode (EncL1,INPUT);
  pinMode (EncL2,INPUT);
  pullUpDnControl(EncL1,PUD_UP);
  pullUpDnControl(EncL2,PUD_UP);

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


    for (int intensity = 100 ; intensity < 800 ; intensity++)
    {
      pwmWrite (PWM_pin_1, (intensity + (corr * int(this->corr_left_))  * this->safe_)) ;	
      pwmWrite (PWM_pin_2, (intensity + (corr * int(this->corr_right_)) * this->safe_)) ;	
      ROS_DEBUG_STREAM_THROTTLE(0.5,"init" << (intensity + (corr * int(this->corr_left_)) * this->safe_) << " " << (intensity + (corr * int(this->corr_right_)) * this->safe_));
      ROS_DEBUG_STREAM_THROTTLE(0.2,"init" << velR1 << " " << velR2 << " " << velL1 << " " << velL2 << "\n ");
      delay (3) ;
    }
    
    for (int tm = 0 ; tm <= 1000 ; tm++)
    {
      pwmWrite (PWM_pin_1, (800 + (corr * int(this->corr_left_)) * this->safe_)) ;	
      pwmWrite (PWM_pin_2, (800 + (corr * int(this->corr_right_)) * this->safe_)) ;	
      ROS_DEBUG_STREAM_THROTTLE(0.5,"Stall" << (800 + (corr * int(this->corr_left_)) * this->safe_) << " " << (800 + (corr * int(this->corr_right_)) * this->safe_));
      ROS_DEBUG_STREAM_THROTTLE(0.2,"init" << velR1 << " " << velR2 << " " << velL1 << " " << velL2 << "\n ");
      delay (10) ;
    }

    for (int intensity = 800 ; intensity >= 100 ; intensity--)
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
