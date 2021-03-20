#include<pkg_movimento/movimento_mode.h>
#include<pkg_movimento/pins_def.h>



void platform_run::safety_task()
{

  while (1)
  {
     this->safe_ = digitalRead(bumper_pin);
     delay(5);
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
  
  if (wiringPiSetup () == -1)
    exit (1) ;

  pinMode (PWM_pin_1, PWM_OUTPUT) ; 
  pinMode (PWM_pin_2, PWM_OUTPUT) ; 
  pinMode (PWM_dir_1, OUTPUT) ;
  pinMode (PWM_dir_2, OUTPUT) ;

  pinMode (bumper_pin, INPUT) ;
  pullUpDnControl(bumper_pin,PUD_UP);

  //TODO in base a dove sei
  digitalWrite(PWM_dir_1,HIGH);
  digitalWrite(PWM_dir_2,HIGH);

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
  

  while (this->curr_pos_ < this->open_dist_)
  {

    this->pwm_current_ = (  abs(this->curr_pos_ - this->open_dist_) < this->ramp_lenght_  ) ? (abs(this->curr_pos_ - this->open_dist_)/this->ramp_lenght_) * this->pwm_max_ : this->pwm_max_;

    pwmWrite (PWM_pin_1, this->pwm_current_ * this->safe_) ;	
    pwmWrite (PWM_pin_2, this->pwm_current_ * this->safe_) ;	
      //std::cout << intensity*safe << "  " << forw <<  std::endl;
    this->rate_.sleep() ;
    
  }	
}

int main (void)
{      


  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);

  // std::thread t1(safety_task);

  // //std::cout << " asd0 " << forw <<  std::endl;
  // t1.detach();
  // //std::cout << " asd1 " << forw <<  std::endl;
  // //t1.join();

  // std::cout << " asd2 " << forw <<  std::endl;



  // while (1)
  // {
	
  //   for (intensity = 0 ; intensity < 1024 ; ++intensity)
  //   {
  //     pwmWrite (PWM_pin_1, intensity*safe) ;	
  //     pwmWrite (PWM_pin_2, intensity*safe) ;	
  //     //std::cout << intensity*safe << "  " << forw <<  std::endl;
  //     delay (ramp_delay) ;
  //   }
  //   delay(mid_delay);

  //   for (intensity = 1023 ; intensity >= 0 ; --intensity)
  //   {
  //     pwmWrite (PWM_pin_1, intensity*safe) ;	
  //     pwmWrite (PWM_pin_2, intensity*safe) ;	
  //     //std::cout << intensity*safe << "  " << forw <<  std::endl;
  //     delay (ramp_delay) ;
  //   }
  //   switch_direction();
  //   delay(stop_time);
  // }	
}
