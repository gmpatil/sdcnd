#ifndef ROAD_H
#define ROAD_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

class Road
{
public:
  const int ego_key = -1;
  const int num_lanes = 3;
  const int speed_limit = 49.5;
  const int MAX_ACCEL = 0.224; // 5m/sec2

  int vehicles_added = 0;

  map<int, Vehicle> vehicles;

  Road(vector<double> wp_x, vector<double> wp_y, vector<double> wp_s, vector<double> wp_dx, vector<double> wp_dy);
  virtual ~Road();

  void update(const json &jsn);
  Vehicle get_ego();
  void populate_traffic();
  void advance();
  void add_ego(int lane_num, int s, vector<int> config_data);
  void cull();
  
  //TODO remove
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
private:
  vector<double> _wp_x;
  vector<double> _wp_y;
  vector<double> _wp_s;
  vector<double> _wp_dx;
  vector<double> _wp_dy;  

  //TODO remove
  int lane = 1;
  double ref_vel = 0.0; // 49.50;

};

#endif // ROAD_H
