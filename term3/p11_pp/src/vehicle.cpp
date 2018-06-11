#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "include/vehicle.h"
#include "include/utils.h"
#include "include/cost.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle() {
}

Vehicle::Vehicle(double x1, double y1, double vx1, double vy1, double s1, 
              double d1, double yaw1, double yaw_rel_lane1){
  this->x = x1;
  this->y = y1;
  this->vx = vx1;
  this->vy = vy1;
  this->s = s1;
  this->d = d1;
  this->yaw = yaw1;

  this->v = sqrt(vx * vx + vy * vy);
  this->yaw_rel_lane = yaw_rel_lane1;
  this->v_s = v * cos(yaw_rel_lane);
  this->v_d = v * sin(yaw_rel_lane);
  
}

Vehicle::~Vehicle() {
}

void Vehicle::update(double x1, double y1, double vx1, double vy1, double s1, 
            double d1, double yaw1, double yaw_rel_lane1) {
  this->x = x1;
  this->y = y1;
  this->vx = vx1;
  this->vy = vy1;
  this->s = s1;
  this->d = d1;
  this->yaw = yaw1;

  this->v = sqrt(vx * vx + vy * vy);
  this->yaw_rel_lane = yaw_rel_lane1;
  this->v_s = v * cos(yaw_rel_lane);
  this->v_d = v * sin(yaw_rel_lane);  
}

vector<string> Vehicle::successor_states() {
  /*
  Provides the possible next states given the current state for the FSM 
  discussed in the course, with the exception that lane changes happen 
  instantaneously, so LCL and LCR can only transition back to KL.
   */
  vector<string> states;
  states.push_back("KL");
  string state = this->state;
  if (state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    if (lane != lanes_available - 1) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state.compare("PLCR") == 0) {
    if (lane != 0) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
  //If state is "LCL" or "LCR", then just return "KL"
  return states;
}


bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
  /*
  Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
  rVehicle is updated if a vehicle is found.
   */
  int min_s = this->goal_s;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector < Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
  /*
  Sets state and kinematics for ego vehicle using the last state of the trajectory.
   */
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s = next_state.s;
  this->v = next_state.v;
  this->a = next_state.a;
}

void Vehicle::configure(vector<int> road_data) {
  /*
  Called by simulator before simulation begins. Sets various
  parameters which will impact the ego vehicle. 
   */
  target_speed = road_data[0];
  lanes_available = road_data[1];
  goal_s = road_data[2];
  goal_lane = road_data[3];
  max_acceleration = road_data[4];
}