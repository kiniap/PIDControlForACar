/*
 * Twiddle.cpp
 *
 *  Created on: Apr 5, 2018
 *      Author: Ananth Kini
 *
 *  Implemented a Generic Twiddle Algorithm
 */

#include <iostream>
#include "Twiddle.h"

const unsigned PENALTY_MULTIPLIER = 10;

Twiddle::Twiddle(const std::vector<double>& initP, double tol, unsigned maxIter, bool enableTwiddle):
m_p(initP),
m_dp(initP.size(), 0.1),
m_runState(Initialized),
m_errorSum(0.0),
m_bestError(0.0),
m_sum_dp(0.0),
m_tolerance(tol),
m_pIndex(0),
m_nIterations(0),
m_maxIterations(maxIter),
m_resetSimulator(false),
m_enableTwiddle(enableTwiddle),
m_penalize(false)
{
  // match dp to the size of initP
  //unsigned size = initP.size();
  //m_dp.resize(size, 1.0);

  // Display initial values
  // Display updated parameter values
  std::cout << "----------------- Twiddle run state: "  << m_runState << "-----------------------" << std::endl;
  std::cout << "Best Error: " << m_bestError << std::endl;
  std::cout << "Initial parameter values ..." << std::endl;
  //std::cout << " m_p.size = " << m_p.size() << " m_dp.size = " << m_p.size() << std::endl;

  for(unsigned i=0; i < m_p.size(); ++i)
  {
    std::cout << m_p[i] << " ";
  }

  std::cout << "\n";

  for(unsigned j=0; j < m_dp.size(); ++j)
  {
    std::cout << m_dp[j] << " ";
  }

  std::cout << "\n";

}

Twiddle::~Twiddle()
{
  // do nothing
};

/*
# Make this tolerance bigger if you are timing out!
def twiddle(tol=0.2):
    # Don't forget to call `make_robot` before every call of `run`!
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)
    # TODO: twiddle loop here
    while (sum(dp) > tol):
        for i in range(len(p)):
            orig_p = p[i]
            p[i]+= dp[i]
            robot = make_robot()
            xt,yt,err = run(robot, p)

            if (err < best_err):
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] = max(p[i]-2*dp[i], 0)
                robot = make_robot()
                xt,yt,err = run(robot, p)

                if(err < best_err):
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] = orig_p
                    dp[i] *= 0.9
*/


void Twiddle::displayParameterValues()
{
  std::cout << "P = ";
  for(unsigned i=0; i < m_p.size(); ++i)
  {
    std::cout << m_p[i] << " ";
  }
  std::cout << "\n";

  std::cout << "dp = ";
  for(unsigned i=0; i < m_dp.size(); ++i)
  {
    std::cout << m_dp[i] << " ";
  }
  std::cout << "\n";

}
void Twiddle::runSimulationWithUpdatedParameters()
{
  // set nIerations to zero
  m_nIterations = 0;

  // reset simulator
  m_resetSimulator = true;

  // reset error sum
  m_errorSum = 0;

  // reset penalty
  m_penalize = false;

  // Display updated parameter values
  std::cout << "----------------- Twiddle run state: "  << m_runState << "-----------------------" << std::endl;
  std::cout << "Best Error: " << m_bestError << std::endl;
  std::cout << "Updated parameter values to ..." << std::endl;
  displayParameterValues();
}

void Twiddle::run(double err)
{

  if(m_penalize)
    err *= PENALTY_MULTIPLIER;

  if (m_nIterations < m_maxIterations){
    ++m_nIterations;

    // compute cumulative error
    m_errorSum += err*err;

    // continue running the cycle
    return;
  }

  // compute sum of dp
  for(auto& n : m_dp) m_sum_dp +=n;

  // if sum_dp is < tolerance, (and we have run maxiterations on a cycle), then we have converged on a set of Kp, Ki, Kd that minimized average crosstrack error
  if ((m_sum_dp < m_tolerance)){
    m_runState = TwiddleDone;

    std::cout << "======================= Final set of parameters" << "===========================" << std::endl;
    displayParameterValues();

    // disable twiddle
    m_enableTwiddle = false;

    return;
  }

  double averageError = m_errorSum/m_nIterations;

  // set best_error to average error, if run state is initialized
  if (m_runState == Initialized){
    m_bestError = averageError; // should be set just once in the beginning!
    m_runState = TwiddleStart;
  }


 // Run Twiddle
  switch(m_runState)
  {
    case TwiddleStart:
      m_p[m_pIndex] += m_dp[m_pIndex]; // explore above
      // run simulation with updated parameter from start
      runSimulationWithUpdatedParameters();
      m_runState = TwiddleUpperErrorCheck;
      return;
    case TwiddleUpperErrorCheck:
      if(averageError < m_bestError)
      {
        m_bestError = averageError;
        m_dp[m_pIndex] *= 1.1;
        m_runState = TwiddleStart;
        return;
      } else
      {
        m_p[m_pIndex] = std::max(m_p[m_pIndex]- 2*m_dp[m_pIndex], 0.0); // explore below and don't allow for negative parameters
        // run simulation with updated parameter from start
        runSimulationWithUpdatedParameters();
        m_runState = TwiddleLowerErrorCheck;
        return;
      }
      break;
    case TwiddleLowerErrorCheck:
      if(averageError < m_bestError)
      {
        m_bestError = averageError;
        m_dp[m_pIndex] *= 1.1;
      } else
      {
        m_p[m_pIndex] += m_dp[m_pIndex]; // set it back to the original value
        m_dp[m_pIndex] *= 0.9; // Start making the dp smaller
      }
      // In either case, go back to start of twiddle
      m_runState = TwiddleStart;
      // Move on to the next parameter
      m_pIndex = (m_pIndex+1)%(m_p.size());
      break;
    default:
      std::cout << "None of the switch case statements are satisfied!" << std::endl;
      // do nothing!
  }


}
