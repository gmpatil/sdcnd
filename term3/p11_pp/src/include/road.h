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
    static const int NUM_LANES = 3;
    static constexpr double SPEED_LIMIT = 49.5;
    static constexpr double MAX_ACCEL = 0.224; // 5m/sec2
    static constexpr double MAX_S = 6945.554;    
    
    // Constructor and destructor
    Road(vector<double> wp_x, vector<double> wp_y, vector<double> wp_s, vector<double> wp_dx, vector<double> wp_dy);
    virtual ~Road();

    void update(const json &jsn);
    
    TrajectoryAction choose_ego_next_state(double ego_s, double ego_d, int frame, map<int, Vehicle> &vehicles, Vehicle &ego);    

    //TODO refactor to remove
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    
  private:
    // Map waypoints
    vector<double> _wp_x;
    vector<double> _wp_y;
    vector<double> _wp_s;
    vector<double> _wp_dx;
    vector<double> _wp_dy;  

    Vehicle ego;
    map<int, Vehicle> vehicles;

    map<int, vector<double>> get_traffic_kinematics(map<int, Vehicle> vehicles, Vehicle ego);

    //TODO remove
    int lane = 1;
    double ref_vel = 0.0; // 49.50;
};

#endif // ROAD_H
