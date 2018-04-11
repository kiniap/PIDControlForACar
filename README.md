# PID Control for a Simulated Car
---
PID controllers are used to control the Steering and Throttle for a simulated car driving around a race track. A combination of manual and "Twiddle " techniques are used to tune the PID controllers. The entire tuning process is described below. The goal is to get the car to drive around the loop, while making sure that it does not go into the non-drivable portion of the road


## PID Controller
---
A PID controller is a commonly used feedback controller that is widely used in a number of applications. The input into the controller is an error, which is the difference between the measured value and some desired reference value that we are trying to track. The output is the control value that will minimize the input error.

For e.g. in the case of the steering controller, the car is trying to follow the center line and deviation from the center line is measured as a cross track error, which is fed into the steering PID controller. The output is the magnitude of the steering angle value normalized to [-1,1]

Similarly, for the throttle PID the car is trying to drive at a particular speed and deviation of the speed of the car from this speed is the speed error, which is fed into the throttle PID controller. The output is a throttle/braking value normalized to [-1, 1]

## Equation
---
Control output = Kp*error (P term) + Ki*Integral of Error over time (I term) + Kd * Rate of change of Error (D term)

Kp, Ki, and Kd are the co-efficients that need to be determined for a particular controller using various tuning techniques. This could range from manually tweaking the parameters, to other methods like "Twiddle" (by Sebastian Thrun), stochastic gradient descent, etc.

## Effect of the P, I, and D terms
---

## P term (Kp * error)

The proportional term or P term provides a control input that is proportional to the input error. This has the effect of reacting instantly and proportionally to any error. A large Kp value will cause the error to be minimized quickly, but will result in overshoot on the other side. This repeats causing oscillations.

This was clearly observed while picking starting values of Kp for the Steering controller. Kp values of around 0.1 with Ki and Kd set to 0, caused steady oscillations. Increasing it beyond 0.2 would cause the car to oscillate wildly out of control.

## D term (Kd * Rate of change of error)

The Derivative term or D term will anticipate the rate of change of error and can be used to damp out the oscillations or even completely eliminate the overshoot. A lot of systems typically aim for a critically damped response with no overshoot.

Addition of a D term to the Steering controller helped damp out the oscillations. A Kd value of around 1.0 (with Kp at 0.1, Ki at 0.0) seemed to work reasonably well.

## I term (Ki * Integral of error over time)

The integral term or I term is needed when there is systematic bias or steady state error, which cannot be eliminated by using just the P and D terms. A large I term could also increase overshoot and lead to oscillations.

A small I term of around with Ki = 0.0001 seemed to reduce the average cross track error. But the effect of this term by itself was not clearly identifiable.

## Tuning Process
---

Implemented a generic Twiddle algorithm to tune any number of parameters.

Started with the manually picked Kp = 0.1, Kd = 1.0, and Ki = 0.0001. In order to minimize the time for the parameters to converge kept Ki constant at 0.0001 and supplied only Kp and Kd to Twiddle with dp values set to 0.1 and 0.2 respectively. Obtained a set of Kp and Kd values that allowed the car  to complete the track, but would cross into some of the non-drivable areas and oscillate wildly at times.

Tried running Twiddle with the Ki included, but that did not help.

Derivatives can be noisy, so tried to smooth out the D_errors by using a moving average filter, but that seemed to make the response worse!

Steering output should not be varying too much from one time step to the next. So tried to use a weighted average of previous steering angle and the current steering angles as the steering output, but that did not improve the response.

Finally ended up implementing a Throttle controller to control the speed to a fixed 30mph. Again used Twiddle to come up with the Kp, Ki, and Kd.

Reran the Twiddle on the Steering controller for Kp, Ki, and Kd and came up with the final values:

Kp = 0.15 Kd = 1.06561 Ki = 0.0001

With the Steering controller and the Throttle controller in place the car is able to drive a lap reasonably well with the tires not entering the non-drivable surface. 










## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
