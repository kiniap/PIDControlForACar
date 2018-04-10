#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include "Twiddle.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void resetSimulator(uWS::WebSocket<uWS::SERVER>& ws)
{
  json msgJson;
  std::string msg = "42[\"reset\",{}]";
  std::cout << msg << std::endl;
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

}

int main()
{
  uWS::Hub h;

  // check connection to simulator
  // For a couple of loops after a reset, the simulator is not connected and uses large ctes from previous simulation
  bool isConnected = false;
  unsigned nLoopsSpeedIsZero = 0;

  PID pid;
  // TODO: Initialize the pid variable.
  // values that seem to work well so far
  // const double Kp = 0.1
  // const double Kd = 2.0
  // const double Ki = 0.0001
  const double Kp = 0.1;
  const double Ki = 0.001;
  const double Kd = 2.0;
  const std::vector<double> pInit = {Kp, Kd};
  const double tol = 0.2;
  const unsigned maxIter = 500;
  Twiddle steeringTwiddle = Twiddle(pInit, tol, maxIter, true);
  pid.Init(Kp, Ki, Kd);

  //unsigned nLoops = 0;

  h.onMessage([&pid, &steeringTwiddle, &isConnected, &nLoopsSpeedIsZero](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      //std::cout << "---------Loop Number " << nLoops << "---------" << std::endl;
      //if (nLoops > 100){
        //resetSimulator(ws);
        //nLoops = 0;
      //}

      //++nLoops;

      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          if (isConnected && speed < 0.1)
            nLoopsSpeedIsZero += 1;
          else
            nLoopsSpeedIsZero = 0;

          //std::cout << "IsConnected: " << isConnected << " speed: "  << speed <<  " nLoopsSpeedIsZero: " << nLoopsSpeedIsZero << std::endl;

          if(steeringTwiddle.m_enableTwiddle){

            if (nLoopsSpeedIsZero > 10)
              steeringTwiddle.m_penalize = true;
              //steeringTwiddle.m_nIterations = steeringTwiddle.m_maxIterations-1; // move on to the next simulations

            steeringTwiddle.run(cte);

            if (steeringTwiddle.m_resetSimulator){
              resetSimulator(ws);
              pid.Init(steeringTwiddle.m_p[0],pid.Ki, steeringTwiddle.m_p[1]);
              steeringTwiddle.m_resetSimulator = false;
              //std::cout << "P_error: " << pid.p_error << "I_error:" << pid.i_error << "D_error:" << pid.d_error << std::endl;

              // after a reset set isConnected to false, and keep setting cte to zero until we are connected
              isConnected = false;
            }

            if(!isConnected)
              cte = 0.0;
          }

          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          // DEBUG
          steer_value = std::max(std::min(steer_value, 1.0), -1.0);
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h, &isConnected](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    isConnected = true;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

