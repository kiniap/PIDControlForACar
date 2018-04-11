#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

const unsigned BUFFER_SIZE = 3;
PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;

  d_error_buffer.assign(BUFFER_SIZE,0.0);
  d_buffer_index = 0;

}

void PID::UpdateError(double cte) {

  //static double last_cte = cte;
  //d_error = cte - last_cte;

  // compute moving average of d_error over 5 samples to smooth it out
  //double d_buffer_sum = 0;
  //d_error_buffer[d_buffer_index] = cte - p_error;
  //d_buffer_index = (d_buffer_index+1)%BUFFER_SIZE;
  //for(auto& n : d_error_buffer) d_buffer_sum +=n;
  //d_error = d_buffer_sum/BUFFER_SIZE;

  d_error = cte - p_error;// p_error is still the cte from the last time step
  p_error = cte;
  i_error += cte;

  //last_cte = cte;
}

double PID::TotalError() {
  return (-Kp*p_error-Ki*i_error-Kd*d_error);
}

