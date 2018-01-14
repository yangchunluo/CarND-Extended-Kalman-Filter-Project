#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "fusion_ekf.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// Read a number of float values into the given VectorXd.
void readVectorXd(istringstream &iss, VectorXd &out) {
  for (int i = 0; i < out.size(); i++) {
    float in;
    iss >> in;
    out(i) = in;
  }
}

int main() {
  uWS::Hub h;

  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // Tools class is used to compute the RMSE later
  Tools tools;

  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  h.onMessage([&fusionEKF, &tools, &estimations, &ground_truth]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (!(length && length > 2 && data[0] == '4' && data[1] == '2')) {
      return;
    }

    // Check if the message has data
    auto s = hasData(string(data));
    if (s == "") {
      string msg = "42[\"manual\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      return;
    }
    
    // Check if the event is telemetry
    auto j = json::parse(s);
    string event = j[0].get<string>();
    if (event != "telemetry") {
      return;
    }
    
    // j[1] is the data JSON object
    string sensor_measurment = j[1]["sensor_measurement"];
    
    MeasurementPackage meas_package;
    istringstream iss(sensor_measurment);

    // Reads first element from the current line
    string sensor_type;
    iss >> sensor_type;

    if (sensor_type.compare("L") == 0) {
      // Lidar measurement: px, py
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
    	readVectorXd(iss, meas_package.raw_measurements_);
    } else if (sensor_type.compare("R") == 0) {
      // Radar measurement: ro, theta, ro_dot
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
    	readVectorXd(iss, meas_package.raw_measurements_);
    }
    
    // Read timestamp
    iss >> meas_package.timestamp_;

    // Read groud truth: x_gt, y_gt, vx_gt, vy_gt
    VectorXd gt_values(4);
    readVectorXd(iss, gt_values);
    ground_truth.push_back(gt_values);
    
    // Call ProcessMeasurment(meas_package) for Kalman filter
    fusionEKF.ProcessMeasurement(meas_package);

    // Push the current estimated x,y positon from the Kalman filter's state vector
    VectorXd estimate(4);
    double p_x = fusionEKF.ekf_.x_(0);
    double p_y = fusionEKF.ekf_.x_(1);
    double v1  = fusionEKF.ekf_.x_(2);
    double v2 = fusionEKF.ekf_.x_(3);
    estimate(0) = p_x;
    estimate(1) = p_y;
    estimate(2) = v1;
    estimate(3) = v2;
    estimations.push_back(estimate);

    //VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

    json msgJson;
    msgJson["estimate_x"] = p_x;
    msgJson["estimate_y"] = p_y;
    msgJson["rmse_x"] =  0;
    msgJson["rmse_y"] =  0;
    msgJson["rmse_vx"] = 0;
    msgJson["rmse_vy"] = 0;
    auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
    std::cout << msg << std::endl;
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

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
