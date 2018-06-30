#include "PID.h"
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

	// Initialise update counter
	count = 0;

	// Initialise Errors
	total_cte = 0.00;
	best_err = numeric_limits<double>::max();

	// Initialise deltas
	p = {Kp, Ki, Kd};
	dp = {0.1*Kp, 0.1*Ki, 0.1*Kd};

	// Initialise Coefficient choice to zero (Means Kp)
	param_state = 0;

	// Try increment first
	twiddle = true;
}

void PID::UpdateError(double cte) {

  // Differential error.
  d_error = cte - p_error;

  // Proportional error.
  p_error = cte;

  // Integral error.
  i_error += cte;


  //Total error
  total_cte += cte*cte;

  //counts
  count++;

}

double PID::TotalError() {

  double totalError = -Kp*p_error - Kd*d_error - Ki*i_error;

  // Limit error between 0 and 1
  if (totalError > 1.0)
	  totalError = 1.0;
  else if (totalError < -1.0)
	  totalError = -1.0;

  return totalError;
}

void PID::Twiddle(char Id[])
{

	//get current average error
	double err = total_cte/count;

	// reset total cte as well as the counts
	total_cte = 0;
	count = 0;


	// If current best error is too large best_err will get the current average error
	if(best_err > 1000)
	{
		best_err = err;

		p[0] += dp[0];

		return;
	}

	// Reset errors
	i_error = 0;
	d_error = 0;
	p_error = 0;

	switch (param_state) {
	      case 0: {
	    	  if(err < best_err){
	    		  //best error becomes the current average error
	    		  best_err = err;
	    		  //increment Kp
	    		  dp[0] *= 1.1;
	    		  //adjust the state recurrently
	    		  param_state = (param_state+1)%3;
	    		  twiddle = true;
	    		  printf("\n Current Best %s Parameter Set Kp,Ki,Kd: %.06f,%.06f,%.06f", Id, p[0], p[1], p[2]);
	    	  }
	    	  else{
	    		  if(twiddle)
	    		  	    twiddle = false;
	    		  		else
	    		  		{
	    		  		  //decrement Kp
						  p[0] += dp[0];
						  dp[0] *= 0.9;
	    		  		}
	    	  }
	    	  //when there is no improvement
	    	  if(twiddle) p[0] += dp[0];
	    	  else p[0]-= 2*dp[0];
	        break;
	      }
	      case 1: {
	    	  if(err < best_err){
	    		  //best error becomes the current average error
	    		  best_err = err;
	    		  //increment Ki
	    		  dp[1] *= 1.1;
	    		  //adjust the state recurrently
	    		  param_state = (param_state+1)%3;
	    		  twiddle = true;
	    		  printf("\n Current Best %s Parameter Set Kp,Ki,Kd: %.06f,%.06f,%.06f", Id, p[0], p[1], p[2]);
	    	  }
	    	  else{
	    		  if(twiddle)
	    		  twiddle = false;
	    		  else{
	    			  //decrement Ki
					  p[1] += dp[1];
					  dp[1] *= 0.9;
	    		  }
	    	  }
	    	  //when there is no improvement
	    	  if(twiddle) p[1] += dp[1];
	    	  else p[1]-= 2*dp[1];

	        break;
	      }
	      case 2: {
	    	  if(err < best_err){
	    		  //best error becomes the current average error
	    		  best_err = err;
	    		  //increment Kd
	    	  	  dp[2] *= 1.1;
	    	  	  //adjust the state recurrently
	    	  	  param_state = (param_state+1)%3;
	    	  	  twiddle = true;
				  printf("\n Current Best %s Parameter Set Kp,Ki,Kd: %.06f,%.06f,%.06f", Id, p[0], p[1], p[2]);
	    	  }
	    	  else{
	    		  if(twiddle)
	    		  twiddle = false;
	    		  else{
	    			  //decrement Kd
					  p[2] += dp[2];
					  dp[2] *= 0.9;
	    		  }
	    	  }
	    	  //when there is no improvement
	    	  if(twiddle) p[2] += dp[2];
	    	  else p[2]-= 2*dp[2];

	        break;
	      }
	}
}


