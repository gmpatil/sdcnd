#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "trajectory_action.h"

using namespace std;

class Vehicle {  
  public:
    map<string, int> lane_direction = {
        {"PLCL", 1},
        {"LCL", 1},
        {"LCR", -1},
        {"PLCR", -1}};

    double v;
    double s;

    int id;
    int lane;
    float a;
    float target_speed;
    int lanes_available;
    float max_acceleration;
    int goal_lane;
    int goal_s;

    string state;

    int preferred_buffer = 6; // impacts "keep lane" behavior.

    // Constructors
    Vehicle();
    Vehicle(double x1, double y1, double vx1, double vy1, double s1, double d1, double yaw1, double yaw_rel_lane1);

    Vehicle(int id, double x1, double y1, double vx1, double vy1, double s1, double d1, double yaw1, double yaw_rel_lane1);

    //Destructor
    virtual ~Vehicle();

    void update(double x1, double y1, double vx1, double vy1, double s1, 
              double d1, double yaw1, double yaw_rel_lane1);

    vector<TrajectoryAction> choose_next_state(map<int, vector<TrajectoryAction>> predictions); 

    vector<string> successor_states();

    vector<TrajectoryAction> generate_trajectory(string state, map<int, vector<TrajectoryAction>> predictions);

    vector<float> get_kinematics(map<int, vector<TrajectoryAction>> predictions, int lane);

    vector<TrajectoryAction> constant_speed_trajectory();

    vector<TrajectoryAction> keep_lane_trajectory(map<int, vector<TrajectoryAction>> predictions);

    vector<TrajectoryAction> lane_change_trajectory(string state, map<int, vector<TrajectoryAction>> predictions);

    vector<TrajectoryAction> prep_lane_change_trajectory(string state, map<int, vector<TrajectoryAction>> predictions);

    bool get_vehicle_behind(map<int, vector<TrajectoryAction>> predictions, int lane, Vehicle &rVehicle);

    bool get_vehicle_ahead(map<int, vector<TrajectoryAction>> predictions, int lane, Vehicle &rVehicle);

    vector<TrajectoryAction> generate_predictions(int horizon = 2);
    
    float position_at(int t);
    
    void realize_next_state(vector<TrajectoryAction> trajectory);

    void configure(vector<int> road_data);

    double get_s(int frame);
    
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