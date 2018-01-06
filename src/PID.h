#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  int cnt;
  double avgerr;
  double maxerr;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  bool is_first;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double MaximumError();

  int NumSteps();

  /*
  ** compute steering angle
  */
  double computeSteering(double cte, double speed); 

};

#endif /* PID_H */
