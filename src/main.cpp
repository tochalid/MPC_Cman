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

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  // Set debug log decimal
  std::cout << std::fixed;
  std::cout << std::setprecision(5);

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << endl;
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];  // waypoint global x coord
          vector<double> ptsy = j[1]["ptsy"];  // waypoint global y coord
          double px = j[1]["x"];               // car's global x coord
          double py = j[1]["y"];               // car's global y coord
          double psi = j[1]["psi"];  // car's global heading angle (rad)
          double v = j[1]["speed"];  // current speed in mph
          int num_waypoints = ptsx.size();

          /*
          * REVIEW: Calculate steering angle and throttle using MPC.
          * Both are in between [-1, 1].
          */

          // Switch velocity from mph into m/s
          v *= (1609.34 / 3600);  // 0.44704

          // Fetch current steer and throttle values
          double delta = j[1]["steering_angle"];  // in radians
          // flip direction from simulator to match motion equations
          delta *= -1;  

          // Throttle value is in [-1, 1]
          double a = j[1]["throttle"];

          // Transform the waypoints from sim to car coordinates
          double y_rotation = sin(-psi);
          double x_rotation = cos(-psi);
          for (int i = 0; i < num_waypoints; ++i) {
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;

            // Rotate the waypoints clockwise by psi
            // https://en.wikipedia.org/wiki/Transformation_matrix
            ptsx[i] = (shift_x * x_rotation - shift_y * y_rotation);
            ptsy[i] = (shift_x * y_rotation + shift_y * x_rotation);
          }
          // Prepare waypoints arrays in Eigen vector typedef for MPC::Solve
          double *ptr_ptsx = &ptsx[0];
          double *ptr_ptsy = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsx_carcoord(ptr_ptsx, num_waypoints);
          Eigen::Map<Eigen::VectorXd> ptsy_carcoord(ptr_ptsy, num_waypoints);

          // Fit 3rd order polyline on waypoints
          auto coeffs = polyfit(ptsx_carcoord, ptsy_carcoord, 3);
          //cout << "COEFFS: " << coeffs << endl;

          // Set CTE at x = 0 (shifted)
          double cte = polyeval(coeffs, 0);
          // Set heading error with psi = 0 (rotated)
          double epsi = -atan(coeffs[1]);

          // Use kinematic equations and state to consider latency
          double latency = 0.1;  // set latency to 100 ms
          const double Lf = 2.67;

          double x_latency = v * latency;
          double y_latency = 0;
          double psi_latency = v * delta / Lf * latency;
          double v_latency = v + a * latency;
          double cte_latency = cte + v * sin(epsi) * latency;
          double epsi_latency = epsi + psi_latency;

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;
          state << x_latency, y_latency, psi_latency, v_latency, cte_latency,
              epsi_latency;
          //cout << "STATE: " << state << endl;

          // Feed state to the MPC solver
          auto solution = mpc.Solve(state, coeffs);
          //cout << "SOLUTION_0: " << solution[0] << endl;
          //cout << "SOLUTION_1: " << solution[1] << endl;

          // Construct the message to send back to the simulator
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          json msgJson;
          double steer_value;
          double throttle_value;
          //flip steering and normalize
          steer_value = -solution[0] / deg2rad(25);  
          // REVIEW:
          throttle_value = solution[1];

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory (GREEN line)
          msgJson["mpc_x"] = mpc.x_vals;
          msgJson["mpc_y"] = mpc.y_vals;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          // Determine the reference line
          // Push polynomial coordinates fitted with waypoints
          int n_points = 16; 
          double increment = 2;
          for (int i = 1; i < n_points; ++i) {
            next_x_vals.push_back(i * increment);
            next_y_vals.push_back(polyeval(coeffs, i * increment));
          }

          // Display the fitted trajectory (YELLOW line)
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // std::cout << "Steer Angle: "<< rad2deg(-solution[0]) << " Throttle
          // Value: " << throttle_value << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
    std::cout << "&h" << &h << std::endl;
    std::cout << "req" << &req << std::endl;
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
