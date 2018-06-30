#ifndef PID_H
#define PID_H
#include <vector>



class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  int count;
  double total_cte;
  double best_err;
  int param_state;
  bool twiddle;

  std::vector<double> p;
  std::vector<double> dp;

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

  /*
   * Twiddle parameter optimization.
  */
  void Twiddle(char Id[]);
};

#endif /* PID_H */
