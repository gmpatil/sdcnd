#include <iostream>
#include "include/road.h"
#include "include/vehicle.h"
#include "include/spline.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>

using json = nlohmann::json;


// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}

double deg2rad(double x) {
  return x * pi() / 180;
}

double rad2deg(double x) {
  return x * 180 / pi();
}

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1)*(x2 - x1)+(y2 - y1)*(y2 - y1));
}

/*
 Returns the maps' index, which is closest way point for global (x, y) co-ordinates.
 */

int closestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

  double closestLen = 100000; //large number
  int cWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      cWaypoint = i;
    }

  }

  return cWaypoint;

}

int nextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
        const vector<double> &maps_y) {

  int cWaypoint = closestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[cWaypoint];
  double map_y = maps_y[cWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = min(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    cWaypoint++;
    if (cWaypoint == maps_x.size()) {
      cWaypoint = 0;
    }
  }

  return cWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates

vector<double> getFrenet(double x, double y, double theta,
        const vector<double> &maps_x, const vector<double> &maps_y) {
  int next_wp = nextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y

vector<double> getXY(double s, double d, const vector<double> &maps_s,
        const vector<double> &maps_x, const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
          (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};

}

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

Vehicle Road::get_ego() {

  return this->vehicles.find(this->ego_key)->second;
}

void Road::populate_traffic() {
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
    ref_vel -= 0.224; // 5m/sec2
  } else if (ref_vel < 49.5) {
    ref_vel += 0.224; // 5m/sec2
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

void Road::advance() {

//  map<int, vector<Vehicle> > predictions;
//
//  map<int, Vehicle>::iterator it = this->vehicles.begin();
//  while (it != this->vehicles.end()) {
//    int v_id = it->first;
//    vector<Vehicle> preds = it->second.generate_predictions();
//    predictions[v_id] = preds;
//    it++;
//  }
//  it = this->vehicles.begin();
//  while (it != this->vehicles.end()) {
//    int v_id = it->first;
//    if (v_id == ego_key) {
//      vector<Vehicle> trajectory = it->second.choose_next_state(predictions);
//      it->second.realize_next_state(trajectory);
//    } else {
//      it->second.increment(1);
//    }
//    it++;
//  }
}

void Road::add_ego(int lane_num, int s, vector<int> config_data) {

  map<int, Vehicle>::iterator it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
    int v_id = it->first;
    Vehicle v = it->second;
    if (v.lane == lane_num && v.s == s) {
      this->vehicles.erase(v_id);
    }
    it++;
  }

  //Vehicle ego = Vehicle(lane_num, s, this->lane_speeds[lane_num], 0);
  Vehicle ego = Vehicle();
  ego.configure(config_data);
  ego.state = "KL";
  this->vehicles.insert(std::pair<int, Vehicle>(ego_key, ego));

}

