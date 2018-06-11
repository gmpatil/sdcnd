#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle
{
public:
  map<string, int> lane_direction = {
      {"PLCL", 1},
      {"LCL", 1},
      {"LCR", -1},
      {"PLCR", -1}};

  double v;
  double s;

  int L = 1;
  int lane;
  float a;
  float target_speed;
  int lanes_available;
  float max_acceleration;
  int goal_lane;
  int goal_s;
  
  string state;


  // Constructors
  Vehicle();
  Vehicle(double x1, double y1, double vx1, double vy1, double s1, 
            double d1, double yaw1, double yaw_rel_lane1);
  
  //Destructor
  virtual ~Vehicle();

  void update(double x1, double y1, double vx1, double vy1, double s1, 
            double d1, double yaw1, double yaw_rel_lane1);
  
  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  void update_s(double dt);

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle &rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle &rVehicle);

  vector<Vehicle> generate_predictions(int horizon = 2);

  void realize_next_state(vector<Vehicle> trajectory);

  void configure(vector<int> road_data);

  private: 
    double x;
    double y;
    double vx;
    double vy;
    double d;  
    double yaw; 
    double yaw_rel_lane;
    double v_s;
    double v_d;
    
};

#endif