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

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object

          // waypoints
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          // current car position/coordinate
          double px = j[1]["x"];
          double py = j[1]["y"];

          // current car orientation
          double psi = j[1]["psi"];

          // current car velocity
          double v = j[1]["speed"];

          // Speed in m/s
          //v *= 0.447;

          // Steering angle in radians
          double delta = j[1]["steering_angle"];

          // Aligning simulator steering convention with MPC
          //delta *= -1.0;
          
          // Throttle
          double a = j[1]["throttle"];
          
          // 100 ms (0.1 s) actuation latency
          double latency = 0.1;
          
          // Car's length from the front to center of gravity
          double Lf = 2.67;

          // convert trajectory/world coordinates to car coordinates
          for(unsigned int i = 0; i < ptsx.size(); i++){
            // shifting waypoints to car origin
            double shifted_x = ptsx[i] - px;
            double shifted_y = ptsy[i] - py;
            //rotate variables into reference frame of vehicle
            double angle = -psi;
            // rotating waypoints to car's x-axis (heading)
            ptsx[i] = shifted_x * cos(angle) - shifted_y * sin(angle);
            ptsy[i] = shifted_x * sin(angle) + shifted_y * cos(angle);
          }

          // convert points from c++ standard vector to Eigen vector
          Eigen::VectorXd ptsx_eigen(ptsx.size());
          Eigen::VectorXd ptsy_eigen(ptsy.size());

          for(unsigned int i = 0; i < ptsx.size(); i++){
            ptsx_eigen[i] = ptsx[i];
            ptsy_eigen[i] = ptsy[i];
          }

          // fit waypoints to 3rd order polynomial
          auto fitted_poly = polyfit(ptsx_eigen, ptsy_eigen,3);

          // calc Cross Track Error(CTE)
          double cte = polyeval(fitted_poly,0.0);

          // calc PSI error
          double epsi = -atan(fitted_poly[1]);
          
          // apply latency
          double latency_x = v * latency;
          double latency_y = 0;
          double latency_psi = v * -delta / Lf * latency;  
          double latency_v = v + a * latency;
          double latency_cte = cte + v * sin(epsi) * latency;
          double latency_epsi = epsi + v * -delta / Lf * latency;

          // calc state error
          Eigen::VectorXd state(6);
          state << latency_x, latency_y, latency_psi, latency_v, latency_cte, latency_epsi;

          // future state
          auto vars = mpc.Solve(state, fitted_poly);

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value;
          double throttle_value;

          //steer_value = -vars[0] / ((25.0 / 180.0) * M_PI);
          steer_value = vars[0];
          throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for(unsigned int i=2; i < vars.size(); ++i){
            if(i % 2 == 0){
              mpc_x_vals.push_back(vars[i]);
            }else{
              mpc_y_vals.push_back(vars[i]);
            }
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double spacing = 5.0;
          int num_points = 10;

          for(int i = 1; i <= num_points; ++i ){
            next_x_vals.push_back(spacing * i);
            next_y_vals.push_back(polyeval(fitted_poly,spacing * i));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          //msgJson["next_x"] = ptsx;
          //msgJson["next_y"] = ptsy;

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
