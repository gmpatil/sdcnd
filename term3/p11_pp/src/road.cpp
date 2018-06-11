#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "include/road.h"
#include "include/vehicle.h"
#include "include/spline.h"
#include "include/utils.h"

using json = nlohmann::json;

/**
 * Initializes Road
 */
Road::Road(vector<double> wp_x, vector<double> wp_y, vector<double> wp_s, 
            vector<double> wp_dx, vector<double> wp_dy) {
  this->_wp_x = wp_x;
  this->_wp_y = wp_y;
  this->_wp_s = wp_s;
  this->_wp_dx = wp_dx;
  this->_wp_dy = wp_dy;
}

Road::~Road() {
}

void addOtherVehicles(map<int, Vehicle> vehicles, const json sensor_fusion, 
        double rd_orientation) {
  
  for (unsigned int i = 0; i < sensor_fusion.size(); i++) {
    int id = sensor_fusion[i][0];
    double x = sensor_fusion[i][1];
    double y = sensor_fusion[i][2];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double s = sensor_fusion[i][5];
    double d = sensor_fusion[i][6];
    double yaw = atan2(vy, vx);    
    double yaw_rel_lane = yaw - rd_orientation;
    
    Vehicle newV(x, y, vx, vy, s, d, yaw, yaw_rel_lane);
    
    vehicles.insert(std::pair<int,Vehicle>(id, newV));
  }
}

void updateOtherVehicles(map<int, Vehicle> vehicles, const json sensor_fusion, 
        double rd_orientation) {
  
  for (unsigned int i = 0; i < sensor_fusion.size(); i++) {
    int id = sensor_fusion[i][0];
    double x = sensor_fusion[i][1];
    double y = sensor_fusion[i][2];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double s = sensor_fusion[i][5];
    double d = sensor_fusion[i][6];
    double yaw = atan2(vy, vx);    
    double yaw_rel_lane = yaw - rd_orientation;
    
    //Vehicle newV(x, y, vx, vy, s, d, yaw, yaw_rel_lane);
    std::map<int,Vehicle>::iterator it = vehicles.find(id);
    if (it == vehicles.end()) {
      Vehicle newV(x, y, vx, vy, s, d, yaw, yaw_rel_lane);
      vehicles.insert(std::pair<int,Vehicle>(id, newV));      
      
      printf("Not found %d\n", id);      
    } else {
      Vehicle v = it->second;
      v.update(x, y, vx, vy, s, d, yaw, yaw_rel_lane);
      printf("Found %d\n", id) ;         
    }
  }
}


void Road::update(const json &jsn) {
  // Main car's localization Data
  double car_x = jsn[1]["x"];
  double car_y = jsn[1]["y"];
  double car_s = jsn[1]["s"];
  double car_d = jsn[1]["d"];
  double car_yaw = jsn[1]["yaw"];
  double car_speed = jsn[1]["speed"];

  // Previous path data given to the Planner
  auto previous_path_x = jsn[1]["previous_path_x"];
  auto previous_path_y = jsn[1]["previous_path_y"];
  
  // Previous path's end s and d values 
  double end_path_s = jsn[1]["end_path_s"];
  double end_path_d = jsn[1]["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = jsn[1]["sensor_fusion"];

  double rd_orientation = road_orientation(car_s, _wp_s, _wp_x, _wp_y);
  
  if (this->vehicles.size() <= 1){
    addOtherVehicles(this->vehicles, sensor_fusion, rd_orientation);
  } else {
    updateOtherVehicles(this->vehicles, sensor_fusion, rd_orientation);
  }
  
  //Initialize
  vector<double> next_x_vals_n;
  vector<double> next_y_vals_n;
  this->next_x_vals = next_x_vals_n;
  this->next_y_vals = next_y_vals_n;

  //
  // go around w/o jerk, use spline
  //

  int prev_size = previous_path_x.size();
  if (prev_size > 0) {
    car_s = end_path_s;
  }

  bool too_close = false;

  // find ref vel
  for (int i = 0; i < sensor_fusion.size(); i++) {
    // car in our lane
    float d = sensor_fusion[i][6]; // ith car, 7th param(6) = d
    if (d < (2 + 4 * this->lane + 2) && d > (2 + 4 * lane - 2)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5]; // 6th param s val

      check_car_s += ((double) prev_size * 0.02 * check_speed);
      //check s values greater than mine and s gap
      if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
        // lower ref vel so that we do not crash
        too_close = true;

        if (lane > 0) {
          lane = 0;
        }

      }
    }
  }


  if (too_close) {
    ref_vel -= MAX_ACCEL; // 5m/sec2
  } else if (ref_vel < SPEED_LIMIT) {
    ref_vel += MAX_ACCEL; // 5m/sec2
  }


  vector<double> ptsx;
  vector<double> ptsy;

  // ref states
  double car_x_calc = car_x;
  double car_y_calc = car_y;
  double ref_yaw = deg2rad(car_yaw);

  if (prev_size < 2) {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsy.push_back(prev_car_y);
    
    ptsx.push_back(car_x_calc);
    ptsy.push_back(car_y_calc);
  } else {
    car_x_calc = previous_path_x[prev_size - 1];
    car_y_calc = previous_path_y[prev_size - 1];

    double prev_car_x = previous_path_x[prev_size - 2];
    double prev_car_y = previous_path_y[prev_size - 2];

    ref_yaw = atan2(car_y_calc - prev_car_y, car_x_calc - prev_car_x);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x_calc);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y_calc);
  }

  // add 3 more forward way points. Use Frenet.
  vector<double> next_wp = getXY(car_s + 30, (2 + 4 * lane),
          this->_wp_s, this->_wp_x, this->_wp_y);
  ptsx.push_back(next_wp[0]);
  ptsy.push_back(next_wp[1]);

  vector<double> next_wp2 = getXY(car_s + 60, (2 + 4 * lane),
          this->_wp_s, this->_wp_x, this->_wp_y);
  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp2[1]);

  vector<double> next_wp3 = getXY(car_s + 90, (2 + 4 * lane),
          this->_wp_s, this->_wp_x, this->_wp_y);
  ptsx.push_back(next_wp3[0]);
  ptsy.push_back(next_wp3[1]);

  // Shift the points to car reference 
  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - car_x_calc;
    double shift_y = ptsy[i] - car_y_calc;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // create spline, reference to the Car co-ordinate
  tk::spline spln;
  spln.set_points(ptsx, ptsy); //ptsx and ptsy now have co-ordinate relative to the car.

  //previous path points
  for (int i = 0; i < previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = spln(target_x);
  double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

  double N = (target_dist / (0.02 * ref_vel / 2.24)); //mts per sec
  double delta = (target_x) / N;
  //printf("N =  %f\nDelta = %f\n", N, delta);

  double x_ref = 0;
  double y_ref = 0;
  // fill up the rest of the path
  for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
    x_ref += delta;
    y_ref = spln(x_ref); //y_point;

    // rotate back to normal/global co-ordinates 
    double x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    double y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
    x_point += car_x_calc;
    y_point += car_y_calc;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }  
}
