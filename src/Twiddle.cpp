/*
 * Twiddle.cpp
 *
 *  Created on: Apr 5, 2018
 *      Author: Ananth Kini
 *
 *  Implemented a Generic Twiddle Algorithm
 */

#include <iostream>
#include <math.h> // fabs
#include "Twiddle.h"

const unsigned PENALTY_MULTIPLIER = 10;
const unsigned FIXED_PENALTY = 0.1;

Twiddle::Twiddle(const std::vector<double>& initP, const std::vector<double>& initDp, double tol, unsigned maxIter, bool enableTwiddle):
m_p(initP),
m_dp(initDp),
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

  // Display initial values
  // Display updated parameter values
  std::cout << "----------------- Twiddle run state: "  << m_runState << "-----------------------" << std::endl;
  std::cout << "Initial parameter values ..." << std::endl;
  displayParameterValues();
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

/*
 * Display updated parameter values
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

  std::cout << "Best Error: " << m_bestError << std::endl;

  std::cout << "Sum of dp: " << m_sum_dp << " Tolerance: " << m_tolerance << std::endl;

}

/*
 * Each time the parameters are update, this is called
 */
void Twiddle::runSimulationWithUpdatedParameters()
{
  // Display updated parameter values
  std::cout << "----------------- Twiddle run state: "  << m_runState << "-----------------------" << std::endl;
  std::cout << "Updated parameter values to ..." << std::endl;
  displayParameterValues();

  // set nIerations to zero
  m_nIterations = 0;

  // reset simulator
  m_resetSimulator = true;

  // reset error sum
  m_errorSum = 0;

  // reset sum of dp
  m_sum_dp = 0;

  // reset penalty
  m_penalize = false;
}


/*
 * Execute the Twiddle algorithm
 */
void Twiddle::run(double err)
{

  if(m_penalize){
    err *= PENALTY_MULTIPLIER;
    std::cout << "Stopped: Applying penalty Multiplier. CTE = " << err << std::endl;
  }

//  // penalize zero crossings **This did not help
// static double last_err = err;
//  if (last_err * err < 0){
//    err = fabs(err) + FIXED_PENALTY;
//    std::cout << "Zero Crossing: Applying fixed penalty. CTE = " << err << std::endl;
//  }
//  last_err = err;

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

 /*
  * Run Twiddle
  */
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
        m_p[m_pIndex] = std::max(m_p[m_pIndex] - 2*m_dp[m_pIndex], 0.0); // explore below and don't allow for negative parameters
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
        // Move on to the next parameter
        m_pIndex = (m_pIndex+1)%(m_p.size());
      }
      // In either case, go back to start of twiddle
      m_runState = TwiddleStart;

      break;
    default:
      std::cout << "None of the switch case statements are satisfied!" << std::endl;
      // do nothing!
  }
}
