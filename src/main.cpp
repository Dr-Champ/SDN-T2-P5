#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Evaluate the desired psi (tangent of the line at x)
double computeTangent(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += i * coeffs[i] * pow(x, i - 1);
  }
  return atan(result);
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

/*
 * Transform a point in global coordinate to car's coordinate given
 * (x_g, y_g) the car's location in global coordinate
 * (x_p, y_p) the point in global coordinate to transform to car's
 * psi_g the car's orientation in global coordinate (measured from +x in ccw)
 * (x_c, y_c) the point in car's coordinate
 * 
 * See https://discussions.udacity.com/t/mpc-car-space-conversion-and-output-of-solve-intuition/249469/12?u=ktawut
 */
void transformGlobalToCarCoordinates(double x_g, double y_g, double psi_g, 
  double x_p, double y_p, double& x_c, double& y_c) {
  x_c = (x_p - x_g) * cos(psi_g) + (y_p - y_g) * sin(psi_g);
  y_c = (y_p - y_g) * cos(psi_g) - (x_p - x_g) * sin(psi_g);
}

/*
* `ptsx` (Array<float>) - The global x positions of the waypoints.
* `ptsy` (Array<float>) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity
since y is the up-down direction.
* `psi` (float) - The orientation of the vehicle in **radians** converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
* `psi_unity` (float) - The orientation of the vehicle in **radians**. This is an orientation commonly used in [navigation](https://en.wikipedia.org/wiki/Polar_coordinate_system#Position_and_navigation).
* `x` (float) - The global x position of the vehicle.
* `y` (float) - The global y position of the vehicle.
* `steering_angle` (float) - The current steering angle in **radians**.
* `throttle` (float) - The current throttle value [-1, 1].
* `speed` (float) - The current velocity in **mph**.
*/

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // Define some constants
    int POLY_ORDER = 4;
    int MAX_STEERING_DEGREE = 25;
    double MPH2MPS = 0.44704;
    int ACTUATOR_LATENCY_MS = 100;
    double DESIRED_SPEED = 9;
    double Lf = 2.67;

    mpc.maxSteeringRad = deg2rad(MAX_STEERING_DEGREE);

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << endl << "=============  New message  ==============" << endl;
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double acceleration = j[1]["throttle"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * - Transform all global coordinates to local (car's) coordinates. This is to
          *   prevent overflowing and (should) make the computation more stable.
          * - Transform velocity from mph to mps
          * - Estimate the states in the next ACTUATOR_LATENCY_MS to take into account
          *   actuator's latency
          * - Compute polyfit coef
          * - Call MPC to solve actuator signals and predicted path. 
          *
          */

          // Convert mph to mps
          double v_mps = v * MPH2MPS;

          // Estimate what car's state will be ACTUATOR_LATENCY_MS later in global
          // coordinate
          px = px + v_mps * cos(psi) * (ACTUATOR_LATENCY_MS / 1000.0);
          py = py + v_mps * sin(psi) * (ACTUATOR_LATENCY_MS / 1000.0);
          psi = psi + v_mps * (ACTUATOR_LATENCY_MS / 1000.0) * delta / Lf;
          v_mps = v_mps + acceleration * (ACTUATOR_LATENCY_MS / 1000.0);

          // Transform waypoints from global to car's coordinate
          Eigen::VectorXd ptsx_car(ptsx.size());
          Eigen::VectorXd ptsy_car(ptsy.size());

          for (int i = 0; i < ptsx.size(); i++) {
            double x_c;
            double y_c;
            transformGlobalToCarCoordinates(px, py, psi, ptsx[i], ptsy[i], x_c, y_c);
            ptsx_car[i] = x_c;
            ptsy_car[i] = y_c;
          }

          // Fit the polynomial, and compute cte and epsi. 
          //std::cout << "ptsx_car: " << ptsx_car << std::endl;
          //std::cout << "ptsy_car: " << ptsy_car << std::endl;
          Eigen::VectorXd coeffs = polyfit(ptsx_car, ptsy_car, POLY_ORDER);

          // defined as f(x) - y
          double cte = polyeval(coeffs, 0) - 0.0;

          // defined as psi - psides
          double epsi = 0.0 - computeTangent(coeffs, 0.0);

          // Create the state vector
          Eigen::VectorXd state(6);
          state << 0.0, 0.0, 0.0, v_mps, cte, epsi;
          //std::cout << "coeffs: " << coeffs << std::endl;
          //std::cout << "state: " << state << std::endl;

          // Solve for optimum actuator signals
          auto vars = mpc.Solve(state, coeffs, DESIRED_SPEED);
          double steer_value = vars[0];
          double throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          // Also, we need to flip the sign of the steering angle as the simulator interprets
          // positive value as turning right (cw)
          msgJson["steering_angle"] = -1 * steer_value / deg2rad(MAX_STEERING_DEGREE);
          msgJson["throttle"] = throttle_value;

          // The MPC predicted trajectory (green)
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // The waypoints/reference line from fitted polynomial (yellow)
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          // These coords are obtained from evaluating the fitted polynomial.
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < mpc.nSteps; i++) {
            mpc_x_vals.push_back(vars[2 + i]);
            mpc_y_vals.push_back(vars[2 + mpc.nSteps + i]);

            next_x_vals.push_back(1.5 * i);
            next_y_vals.push_back(polyeval(coeffs, 1.5 * i));
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(ACTUATOR_LATENCY_MS));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          cout << "End processing 42 ..." << endl;
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
