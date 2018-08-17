#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "include/vehicle.h"
#include "include/utils.h"
#include "include/cost.h"
#include "include/trajectory_action.h"

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
  
  this->lane = (int) std::floor(this->d / 4.0);
  this->goal_lane = this->lane ;
}

Vehicle::Vehicle(int idt, double x1, double y1, double vx1, double vy1, double s1, 
              double d1, double yaw1, double yaw_rel_lane1){
  this->id = idt;
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
  
  this->lane = (int) std::floor(this->d / 4.0);
  this->goal_lane = this->lane ;
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
  
  this->lane = (int) std::floor(this->d / 4.0);
  this->goal_lane = this->lane ;
}


TrajectoryAction Vehicle::choose_next_state(map<int, TrajectoryAction> predictions, int horizon) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<string> final_states;
    vector<TrajectoryAction> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        TrajectoryAction trajectory = generate_trajectory(*it, predictions);
        if (&trajectory != &NULL_TRAJECTORY_ACTION) {
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    return final_trajectories[best_idx];
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

TrajectoryAction Vehicle::generate_trajectory(string state, map<int, TrajectoryAction> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    TrajectoryAction trajectory;
    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }
    return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, TrajectoryAction> predictions, int lane) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    float max_velocity_accel_limit = this->max_acceleration + this->v;
    float new_position;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
        } else {
            float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
        }
    } else {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
    }
    
    new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + new_velocity + new_accel / 2.0;
    return{new_position, new_velocity, new_accel};
    
}

bool Vehicle::get_vehicle_behind(map<int, TrajectoryAction> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int max_s = -1;
    bool found_vehicle = false;
    TrajectoryAction temp_vehicle;
    for (map<int, TrajectoryAction>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second;
        if (temp_vehicle.goalLane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            // rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}


bool Vehicle::get_vehicle_ahead(map<int, TrajectoryAction> predictions, int lane, Vehicle & rVehicle) {
  /*
  Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
  rVehicle is updated if a vehicle is found.
   */
  int min_s = this->goal_s;
  bool found_vehicle = false;
  TrajectoryAction temp_vehicle;
  for (map<int, TrajectoryAction>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second;
    if (temp_vehicle.goalLane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
      min_s = temp_vehicle.s;
      //rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

void Vehicle::realize_next_state(TrajectoryAction trajectory) {
  /*
  Sets state and kinematics for ego vehicle using the last state of the trajectory.
   */
  TrajectoryAction next_state = trajectory;
  //this->state = next_state.state;
  this->lane = next_state.goalLane;
  this->s = next_state.s;
  //this->v = next_state.v;
  //this->a = next_state.a;
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


double Vehicle::get_s(int frame) {
  double frame_s = this->s + ((double) frame * 0.02 * this->v);
  return frame_s;
}

TrajectoryAction Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
    TrajectoryAction trajectory = TrajectoryAction(TrajectoryActionSpeed::MaintainSpeed, false, this->lane);

    return trajectory;
}

TrajectoryAction Vehicle::keep_lane_trajectory(map<int, TrajectoryAction> predictions) {
    /*
    Generate a keep lane trajectory.
    */
//    vector<TrajectoryAction> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
//    vector<float> kinematics = get_kinematics(predictions, this->lane);
//    float new_s = kinematics[0];
//    float new_v = kinematics[1];
//    float new_a = kinematics[2];
//    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    TrajectoryAction trajectory = TrajectoryAction(TrajectoryActionSpeed::MaintainSpeed, false, this->lane);

    return trajectory;
}

TrajectoryAction Vehicle::prep_lane_change_trajectory(string state, map<int, TrajectoryAction> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    TrajectoryAction trajectory = TrajectoryAction(TrajectoryActionSpeed::MaintainSpeed, false, this->lane);
    
//    vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);
//
//    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
//        //Keep speed of current lane so as not to collide with car behind.
//        new_s = curr_lane_new_kinematics[0];
//        new_v = curr_lane_new_kinematics[1];
//        new_a = curr_lane_new_kinematics[2];
//        
//    } else {
//        vector<float> best_kinematics;
//        vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
//        //Choose kinematics with lowest velocity.
//        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
//            best_kinematics = next_lane_new_kinematics;
//        } else {
//            best_kinematics = curr_lane_new_kinematics;
//        }
//        new_s = best_kinematics[0];
//        new_v = best_kinematics[1];
//        new_a = best_kinematics[2];
//    }
//
//    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    return trajectory;
}

TrajectoryAction Vehicle::lane_change_trajectory(string state, map<int, TrajectoryAction> predictions) {
    /*
    Generate a lane change trajectory.
    */
//    int new_lane = this->lane + lane_direction[state];
//    vector<Vehicle> trajectory;
//    Vehicle next_lane_vehicle;
//    //Check if a lane change is possible (check if another vehicle occupies that spot).
//    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
//        next_lane_vehicle = it->second[0];
//        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
//            //If lane change is not possible, return empty trajectory.
//            return trajectory;
//        }
//    }
//    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
//    vector<float> kinematics = get_kinematics(predictions, new_lane);
//    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    
    TrajectoryAction trajectory = TrajectoryAction(TrajectoryActionSpeed::MaintainSpeed, false, this->lane);
    
    return trajectory;
}


float Vehicle::position_at(int t) {
    return this->s + this->v*t + this->a*t*t/2.0;
}


TrajectoryAction Vehicle::generate_predictions(int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
	TrajectoryAction predictions = TrajectoryAction(TrajectoryActionSpeed::MaintainSpeed, false, this->lane);
    predictions.v = this->v;
    predictions.s = (double) this->s + (horizon * 0.02 * this->v) ; 

    return predictions;
}