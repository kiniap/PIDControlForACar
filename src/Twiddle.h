/*
 * Twiddle.h
 *
 *  Created on: Apr 5, 2018
 *      Author: Ananth Kini
 */

#ifndef SRC_TWIDDLE_H_
#define SRC_TWIDDLE_H_

#include <vector>
#include "PID.h"

class Twiddle
{
public:
  enum RunState {
    Initialized=0,
    RunningSimulation,
    TwiddleStart,
    TwiddleUpperErrorCheck,
    TwiddleLowerErrorCheck,
    TwiddleDone,
  };

  // constructor
  explicit Twiddle(const std::vector<double>& initP, const std::vector<double>& initDp, double tol, unsigned maxIter, bool enableTwiddle = true);
  virtual ~Twiddle();
  void displayParameterValues();
  void runSimulationWithUpdatedParameters();
  void run(double err);

  std::vector<double> m_p; // parameters to be tuned
  std::vector<double> m_dp; // amount to vary the parameters by
  RunState m_runState;
  double m_errorSum;
  double m_bestError; // average error over the entire run use to decide best set of parameters
  double m_sum_dp;
  double m_tolerance;
  unsigned m_pIndex; //current parameter being tuned
  unsigned m_nIterations; // divide by parameter size before comparing with max iterations?
  unsigned m_maxIterations; // max iterations to run each cycle for
  bool m_resetSimulator;
  bool m_enableTwiddle;
  bool m_penalize; // penalize for not meeting requirements by setting the average error to a larger number

};



#endif /* SRC_TWIDDLE_H_ */
