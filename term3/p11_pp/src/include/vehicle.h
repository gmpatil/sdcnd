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
  // 1 miles = 1609.34 meters
  public:
    static const int NUM_LANES = 3;
    static constexpr double SPEED_LIMIT = 35; //49.5;
    static constexpr double MAX_ACCEL =  0.15 ; //0.224; // 5m/sec2
    static constexpr double MAX_S = 6945.554;  // 4.32 miles      
    static const int PREF_BUFFER = 30; // 6; // impacts "keep lane" behavior.

    map<string, int> lane_direction = {
        {"PLCL", -1},
        {"LCL", -1},
        {"LCR", 1},
        {"PLCR", 1}};

    double v;
    double s;

    int id;
    int lane;
    int goal_lane;    
    float a;
    double goal_s;
    float goal_d;
    double goal_v;
    
    int goal_horizon;


    string state = "KL";

    // Constructors
    Vehicle();
    //Vehicle(double x1, double y1, double vx1, double vy1, double s1, float d1, double yaw1, double yaw_rel_lane1);
    Vehicle(int id, double x1, double y1, double vx1, double vy1, double s1, float d1, double yaw1, double yaw_rel_lane1);

    //Destructor
    virtual ~Vehicle();

    void update(double x1, double y1, double vx1, double vy1, double s1, float d1, double yaw1, double yaw_rel_lane1);
    void updateGoal(double s, float d, int horizon); //for Ego
    void updateGoal(int horizon); // for non-Ego 

    TrajectoryAction choose_next_state(map<int, TrajectoryAction> &predictions, map<int, vector<double>> &trafficInfo , int horizon); 

    vector<string> successor_states();
    
    TrajectoryAction generate_trajectory(string state, map<int, TrajectoryAction> predictions, map<int, vector<double>> traffic_info);

    vector<float> get_kinematics(map<int, TrajectoryAction> predictions, int lane);

    TrajectoryAction constant_speed_trajectory(map<int, vector<double>> traffic_info);

    TrajectoryAction keep_lane_trajectory(map<int, TrajectoryAction> predictions, map<int, vector<double>> traffic_info);

    TrajectoryAction lane_change_trajectory(string state, map<int, TrajectoryAction> predictions, map<int, vector<double>> traffic_info);

    TrajectoryAction prep_lane_change_trajectory(string state, map<int, TrajectoryAction> predictions, map<int, vector<double>> traffic_info);

    bool get_vehicle_behind(map<int, TrajectoryAction> predictions, int lane, Vehicle &rVehicle);

    bool get_vehicle_ahead(map<int, TrajectoryAction> predictions, int lane, Vehicle &rVehicle);

    TrajectoryAction generate_predictions(int horizon = 2);
    
    float position_at(int t);
    
    void realize_next_state(TrajectoryAction trajectory);

    //void configure(vector<int> road_data);

    double get_s(int frame);
    
  private: 
    double x;
    double y;
    double vx;
    double vy;
    float d;  
    double yaw; 
    double yaw_rel_lane;
    double v_s;
    double v_d;    
};

#endif