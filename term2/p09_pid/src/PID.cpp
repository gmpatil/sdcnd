#include "PID.h"

#include <chrono>
#include <complex>
#include <iostream> 

using namespace std;
using namespace std::chrono;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, int max_steps) {
  this->p_error = 0.0;
  this->d_error = 0.0;
  this->i_error = 0.0;
  
  this->cte_time = std::chrono::milliseconds::zero();
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  this->steering_angle = 0.0;
  this->throttle = 0.0;  
  this->max_steps = max_steps;
}

void PID::UpdateError(double cte) {
  double prev_p_error = this->p_error;
  
  this->p_error = cte;
  this->i_error += this->p_error;
  this->d_error = this->p_error - prev_p_error;
  
  if (this->cte_time > std::chrono::milliseconds::zero()) {
    std::chrono::milliseconds prev_cte_time  = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    
    double time_diff = ((this->cte_time.count() - prev_cte_time.count()) );  
    
    this->steering_angle = -this->Kp * this->p_error 
            - ((this->Kd * this-> d_error)/time_diff ) 
            - this->Ki * this->i_error;  

    this->steps++;
    
    if (steps >= (max_steps/2) ) {
      this->total_error += pow(cte, 2);
    }    
  } else {
    this->cte_time  = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
  }
}

void PID::UpdateError(double cte, double speed) {
  double prev_p_error = this->p_error;
  double prev_steering_angle = this->steering_angle;
  
  this->p_error = cte;
  this->i_error += this->p_error;
  this->d_error = this->p_error - prev_p_error;
  
  if (this->cte_time > std::chrono::milliseconds::zero()) {
    std::chrono::milliseconds prev_cte_time  = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    
    double time_diff = ((this->cte_time.count() - prev_cte_time.count()) );  
    
    this->steering_angle = -this->Kp * this->p_error 
            - ((this->Kd * this-> d_error)/time_diff ) 
            - this->Ki * this->i_error;  

//    //Adjust for lag/time delay
//    
//    if ((prev_steering_angle < 0) && (this->steering_angle < 0)) {
//      if (abs(prev_steering_angle) < abs(this->steering_angle)){
//        this->steering_angle = (this->steering_angle + prev_steering_angle) / 2.0 ;
//      }
//    }
//
//    if ((prev_steering_angle > 0) && (this->steering_angle > 0)) {
//      if (abs(prev_steering_angle) < abs(this->steering_angle)){
//        this->steering_angle = (this->steering_angle + prev_steering_angle) / 2.0 ;
//      }
//    }
    
    // Reduce throttle based on error and steering angle.
    if ((abs(this->p_error) < 0.50) ||
        (abs(this->steering_angle) < 5)) {
      this->throttle = 0.2;
      //std::cout << "throttle 0.2" << std::endl;
    } else {
      if (abs(speed) > 0.2){
          this->throttle = 0.0;
          //std::cout << "throttle 0" << std::endl;
      } else { // Speed is already close to zero throttle up.
        this->throttle = 0.1;
        //std::cout << "throttle 0.1" << std::endl;
      }
    }
  } else {
    this->cte_time  = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
  }
}

double PID::TotalError() {
  return this->total_error;
}

