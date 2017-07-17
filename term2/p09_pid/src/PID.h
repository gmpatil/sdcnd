#ifndef PID_H
#define PID_H

#include <chrono>  

class PID {
public:
  /*
  * Errors
  */
  double p_error; // CTE
  double i_error; // sum(CTE) for t-> 0 to n
  double d_error; // delta (p_error_tn - p_error_tn-1) / delta_t
  
  std::chrono::milliseconds cte_time = std::chrono::milliseconds::zero();
  double steering_angle = 0.0;
  double throttle = 0.0;
  int max_steps = 0;
  int steps = 0;
  double total_error = 0.0;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, int max_steps);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Update the PID error variables given cross track error and speed.
  */
  void UpdateError(double cte, double speed);


  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
