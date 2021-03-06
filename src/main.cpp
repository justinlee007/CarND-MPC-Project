#include <math.h>
#include <uWS/uWS.h>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "Tracker.h"

// for convenience
using json = nlohmann::json;

static const double TIME_DELAY_SEC = 0.1;
static const double LF = 2.67;
static const double POLY_INC = 2.5;
static const int NUM_POINTS = 25;
static const int SAMPLE_SIZE = 100;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned, else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
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
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
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

  MPC mpc;
  Tracker tracker;

  tracker.init(SAMPLE_SIZE);

  h.onMessage([&mpc, &tracker](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::string sdata = std::string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      std::string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          std::vector<double> ptsx = j[1]["ptsx"];
          std::vector<double> ptsy = j[1]["ptsy"];
          double x = ptsx[0];
          double y = ptsy[0];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          double cos_psi = cos(0 - psi);
          double sin_psi = sin(0 - psi);
          for (int i = 0; i < ptsx.size(); i++) {

            // Shift car reference angle to 90 degrees
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;

            ptsx[i] = (shift_x * cos_psi - shift_y * sin_psi);
            ptsy[i] = (shift_x * sin_psi + shift_y * cos_psi);

          }

          double *ptrx = &ptsx[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

          double *ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          // Calculate cte and epsi
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          // Factor in delay
          double delay_x = v * TIME_DELAY_SEC;
          double delay_y = 0;
          double delay_psi = -v * steer_value / LF * TIME_DELAY_SEC;
          double delay_v = v + throttle_value * TIME_DELAY_SEC;
          double delay_cte = cte + v * sin(epsi) * TIME_DELAY_SEC;
          double delay_epsi = epsi - v * steer_value / LF * TIME_DELAY_SEC;

          Eigen::VectorXd state(6);
          state << delay_x, delay_y, delay_psi, delay_v, delay_cte, delay_epsi;

          // Calculate steering angle and throttle using MPC.
          auto vars = mpc.Solve(state, coeffs);

          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          for (int i = 1; i < NUM_POINTS; i++) {
            double next_val = POLY_INC * i;
            next_x_vals.push_back(next_val);
            next_y_vals.push_back(polyeval(coeffs, next_val));
          }

          std::vector<double> mpc_x_vals;
          std::vector<double> mpc_y_vals;

          for (int i = 4; i < vars.size(); i++) {
            if (i % 2 == 0) {
              mpc_x_vals.push_back(vars[i]);
            } else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          bool ok = (bool) vars[0];
          double cost = vars[1];
          if (ok) {
            steer_value = vars[2];
            throttle_value = vars[3];
          } else {
            steer_value = 0;
            throttle_value = -1; // full stop
          }
          tracker.onMessageProcessed(cte, v, throttle_value, cost, x, y);

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          // Manually added latency. The purpose is to mimic real driving conditions where the car does actuate the commands instantly.
          std::this_thread::sleep_for(std::chrono::milliseconds((int) (TIME_DELAY_SEC * 1000)));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
