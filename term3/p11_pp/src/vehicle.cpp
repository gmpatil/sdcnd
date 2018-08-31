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


// Vehicle::Vehicle(double x1, double y1, double vx1, double vy1, double s1, 
//               double d1, double yaw1, double yaw_rel_lane1){
//   this->x = x1;
//   this->y = y1;
//   this->vx = vx1;
//   this->vy = vy1;
//   this->s = s1;
//   this->d = d1;
//   this->yaw = yaw1;

//   this->v = sqrt(vx * vx + vy * vy);
//   this->yaw_rel_lane = yaw_rel_lane1;
//   this->v_s = v * cos(yaw_rel_lane);
//   this->v_d = v * sin(yaw_rel_lane);
  
//   this->lane = (int) std::floor(this->d / 4.0);
//   this->goal_lane = this->lane ;
// }

Vehicle::Vehicle(int idt, double x1, double y1, double vx1, double vy1, double s1, 
              float d1, double yaw1, double yaw_rel_lane1){
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
  this->goal_v = v;  
}


Vehicle::~Vehicle() {
}

void Vehicle::update(double x1, double y1, double vx1, double vy1, double s1, 
            float d1, double yaw1, double yaw_rel_lane1) {
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

void Vehicle::updateGoal(double gs, float gd, int ghorizon) {
    this->d = gd;
    this->goal_d = gd;
    this->goal_s = gs;
    this->goal_horizon = ghorizon;
    this->goal_lane = (int) std::floor(this->goal_d / 4.0);
    this->lane = this->goal_lane;
}

void Vehicle::updateGoal(int ghorizon) {
    this->goal_d = this->d;
    this->goal_s = this->s + this->v * 0.02 * ghorizon; // 0.02 is sampling rate.
    this->goal_horizon = ghorizon;
    this->goal_lane = (int) std::floor(this->goal_d / 4.0);    
}

TrajectoryAction Vehicle::choose_next_state(map<int, TrajectoryAction> &predictions, vector<vector<double>> &traffic_info, int horizon) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    
    cout << "Choosing next state for Ego in Vehicle\n" ;   
    
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<string> final_states;
    vector<TrajectoryAction> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        TrajectoryAction trajectory = generate_trajectory(*it, predictions, traffic_info);
        if (&trajectory != &NULL_TRAJECTORY_ACTION) {
            cost = calculate_cost(*this, predictions, trajectory);
            cout << "State:" << *it << " cost=" << cost << "\n" ;
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        } else {
            std::cout << "NULL traj action for state:" << *it << std::endl;
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);

    cout << "min cost traj:state=" << final_trajectories[best_idx].state << " accelerate(a0/d1/m2):" << (int) final_trajectories[best_idx].speedAction << " lanechange(k0/l1/r2):"<< (int) final_trajectories[best_idx].changeLane << "\n" ;
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
      states.push_back("PLCL");
      states.push_back("LCL");
  } else if (state.compare("PLCR") == 0) {
      states.push_back("PLCR");
      states.push_back("LCR");
  }
  //If state is "LCL" or "LCR", then just return "KL"
  return states;
}

TrajectoryAction Vehicle::generate_trajectory(string state, map<int, TrajectoryAction> predictions, vector<vector<double>> &traffic_info) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    TrajectoryAction trajectory;
    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory(traffic_info);
    } else if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions, traffic_info);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions, traffic_info);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions, traffic_info);
    }
    return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, TrajectoryAction> predictions, int lane) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    float max_velocity_accel_limit = this->MAX_ACCEL + this->v;
    float new_position;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
        } else {
            float max_velocity_in_front = (vehicle_ahead.s - this->s - this->PREF_BUFFER) + vehicle_ahead.v - 0.5 * (this->a);
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), (float)SPEED_LIMIT);
        }
    } else {
        new_velocity = min(max_velocity_accel_limit, (float) SPEED_LIMIT);
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
        if (temp_vehicle.goal_lane == this->lane && temp_vehicle.goal_s < this->goal_s && temp_vehicle.goal_s > max_s) {
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
  int min_s = Vehicle::MAX_S;
  bool found_vehicle = false;
  TrajectoryAction temp_vehicle;
  for (map<int, TrajectoryAction>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second;
    if (temp_vehicle.goal_lane == this->lane && temp_vehicle.goal_s > this->goal_s && temp_vehicle.goal_s < min_s) {
      min_s = temp_vehicle.goal_s;
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
  this->lane = next_state.goal_lane;
  this->s = next_state.s;
  //this->v = next_state.v;
  //this->a = next_state.a;
}

double Vehicle::get_s(int frame) {
  double frame_s = this->s + ((double) frame * 0.02 * this->v);
  return frame_s;
}

TrajectoryAction Vehicle::constant_speed_trajectory(vector<vector<double>> &traffic_info) {
    /*
    Generate a constant speed trajectory.
    */

   vector<double> lane_info = traffic_info[this->goal_lane];
   double ahead_vehicle_s = lane_info[0];
   if (ahead_vehicle_s - this->goal_s <= PREF_BUFFER) {
       return NULL_TRAJECTORY_ACTION;
   } else {
    TrajectoryAction trajectory = TrajectoryAction(TrajectoryActionSpeed::MaintainSpeed, TrajectoryActionLaneChange::KeepLane, this->goal_lane);
    trajectory.s = this->s;
    trajectory.goal_s = this->goal_s;
    trajectory.v = this->goal_v;
    trajectory.state = "KL" ;  

    trajectory.tgt_lane_dist = lane_info[0];        
    trajectory.tgt_lane_vel = lane_info[1];
    trajectory.tgt_lane_coll = lane_info[4];    

    return trajectory;
   }
}

TrajectoryAction Vehicle::keep_lane_trajectory(map<int, TrajectoryAction> predictions, vector<vector<double>> &traffic_info) {
    /*
    Generate a keep lane trajectory.
    */
   vector<double> lane_info = traffic_info[this->goal_lane];
   double dist_ahead_vehicle_s = lane_info[0];

   if ( (dist_ahead_vehicle_s <= PREF_BUFFER)  || (lane_info[4] == 1)){
        cout << "KL Traj: Possible collision, decelerating...." << "\n" ;
        TrajectoryAction trajectory = TrajectoryAction(TrajectoryActionSpeed::Decelerate, 
                TrajectoryActionLaneChange::KeepLane, this->goal_lane);
        trajectory.s = this->s;
        trajectory.goal_s = this->s + dist_ahead_vehicle_s;
        trajectory.v = this->goal_v - MAX_ACCEL;     
        trajectory.state = "KL" ;  

        trajectory.tgt_lane_dist = lane_info[0];        
        trajectory.tgt_lane_vel = lane_info[1];
        trajectory.tgt_lane_coll = lane_info[4];

        cout << "KL: Ego.v=" << this->v <<" traj_v=" << trajectory.v << "\n";        
        return trajectory;
   } else {
        TrajectoryActionSpeed speed;
        double vel;
        if (this->goal_v >= SPEED_LIMIT) {
            speed = TrajectoryActionSpeed::Decelerate;
            vel = this->goal_v  - MAX_ACCEL;       
        } else {
            speed = TrajectoryActionSpeed::Accelerate;
            vel = this->goal_v  + MAX_ACCEL;       
        }

        double tgt_lane_dist = lane_info[0];
        double tgt_lane_vel = lane_info[1];
        double adj_lane_dist = tgt_lane_dist;
        double adj_lane_vel = tgt_lane_vel;

        // int leftLane = this->lane - 1;
        // if (leftLane >= 0) {
        //     adj_lane_vel  = traffic_info[leftLane][1];
        //     adj_lane_dist  = traffic_info[leftLane][0];
        //     // Possible lane change
        //     if ((adj_lane_dist < PREF_BUFFER) && (adj_lane_vel < this->v )) {
        //         tgt_lane_dist = adj_lane_dist;
        //         tgt_lane_vel = adj_lane_vel;
        //         speed = TrajectoryActionSpeed::Decelerate;
        //         vel = this->goal_v  - MAX_ACCEL;                
        //     }
        // }

        // int rightLane = this->lane + 1;
        // if (rightLane < NUM_LANES) {
        //     adj_lane_vel  = traffic_info[rightLane][1];
        //     adj_lane_dist  = traffic_info[rightLane][0];

        //     // Possible lane change
        //     if ((adj_lane_dist < PREF_BUFFER) && (adj_lane_vel < this->v )) {
        //         tgt_lane_dist = adj_lane_dist;
        //         tgt_lane_vel = adj_lane_vel;
        //         speed = TrajectoryActionSpeed::Decelerate;
        //         vel = this->goal_v  - MAX_ACCEL;                
        //     }
        // }        

        TrajectoryAction trajectory = TrajectoryAction(speed, TrajectoryActionLaneChange::KeepLane, this->goal_lane);
        trajectory.s = this->s;
        trajectory.goal_s = this->goal_s;
        trajectory.v = vel;       
        trajectory.state = "KL" ;

        trajectory.tgt_lane_dist = tgt_lane_dist;        
        trajectory.tgt_lane_vel = tgt_lane_vel ;
        trajectory.tgt_lane_coll = lane_info[4];

        cout << "KL: Ego.v=" << this->v << " traj_v=" << trajectory.v << " ttraj.gt_lane_vel=" << trajectory.tgt_lane_vel << "\n";                
        return trajectory;
   }
}

TrajectoryAction Vehicle::prep_lane_change_trajectory(string state, map<int, TrajectoryAction> predictions, vector<vector<double>> &traffic_info) {
    /*
    Generate a trajectory preparing for a lane change.
    */

    // for (int i = 0; i < NUM_LANES; i++)
    // {
    //     vector<double> lane_info = traffic_info[(double)i];
    //     cout << "Lane " << i << " fs:" << lane_info[0] << " fv:" << lane_info[1] << " bs:" << lane_info[2] << " bv:" << lane_info[3] << " coll:" << lane_info[4] << "\n";
    // }

    int new_lane = this->lane + lane_direction[state];
    if (new_lane < 0) {
        new_lane = 0;
    }

    if (new_lane >= NUM_LANES) {
        new_lane = NUM_LANES - 1;
    }

    vector<double> curr_lane_info = traffic_info[this->goal_lane];
    vector<double> tgt_lane_info = traffic_info[new_lane];

    TrajectoryActionSpeed speed = TrajectoryActionSpeed::MaintainSpeed;
    double ahead_vehicle_s = curr_lane_info[0];
    double ahead_vehicle_s_tgt = tgt_lane_info[0];

    if (tgt_lane_info[4] == 1.0) {
        // possible collision
        if (state.compare("PLCL") == 0) {
            cout << "Can not PLCL due to risk of colllision\n";
        } else {
            cout << "Can not PLCR due to risk of colllision\n";
        }

        return NULL_TRAJECTORY_ACTION;
    }

    TrajectoryAction trajectory;

    if ((ahead_vehicle_s <= PREF_BUFFER) || (ahead_vehicle_s_tgt <= PREF_BUFFER)){
        // TrajectoryAction trajectory = TrajectoryAction(TrajectoryActionSpeed::Decelerate, 
        trajectory = TrajectoryAction(TrajectoryActionSpeed::Decelerate,         
                TrajectoryActionLaneChange::KeepLane, this->goal_lane);
        trajectory.s = this->s;
        trajectory.goal_s = this->s + ahead_vehicle_s;
        trajectory.v = this->goal_v - MAX_ACCEL;

        if (trajectory.v > tgt_lane_info[1]) {
            trajectory.v = tgt_lane_info[1] ;
        }

        trajectory.state = state ;       
        trajectory.tgt_lane_dist = (curr_lane_info[0] + tgt_lane_info[0])/2.0;        
        trajectory.tgt_lane_vel = (curr_lane_info[1] + tgt_lane_info[1])/2.0;        
        trajectory.tgt_lane_coll = (curr_lane_info[4] + tgt_lane_info[4])/2.0;
        // return trajectory;
    } else {
        TrajectoryActionSpeed speed;
        double vel = this->goal_v;
        if ((vel >= SPEED_LIMIT) || (vel > tgt_lane_info[1]) || (vel > curr_lane_info[1])) {
            speed = TrajectoryActionSpeed::Decelerate;
            vel = vel - MAX_ACCEL;       
        } else {
            speed = TrajectoryActionSpeed::MaintainSpeed;
        }

        double tgt_lane_dist = curr_lane_info[0];
        double tgt_lane_vel = curr_lane_info[1];
        double adj_lane_dist = tgt_lane_dist;
        double adj_lane_vel = tgt_lane_vel;

        int leftLane = this->lane - 1;
        if (leftLane >= 0) {
            adj_lane_vel  = traffic_info[leftLane][1];
            adj_lane_dist  = traffic_info[leftLane][0];
            // Possible lane change
            if ((adj_lane_dist < PREF_BUFFER) && (adj_lane_vel < this->v )) {
                tgt_lane_dist = adj_lane_dist;
                tgt_lane_vel = adj_lane_vel;
                speed = TrajectoryActionSpeed::Decelerate;
                vel = this->goal_v  - MAX_ACCEL;                
            }
        }

        int rightLane = this->lane + 1;
        if (rightLane < NUM_LANES) {
            adj_lane_vel  = traffic_info[rightLane][1];
            adj_lane_dist  = traffic_info[rightLane][0];

            // Possible lane change
            if ((adj_lane_dist < PREF_BUFFER) && (adj_lane_vel < this->v )) {
                tgt_lane_dist = adj_lane_dist;
                tgt_lane_vel = adj_lane_vel;
                speed = TrajectoryActionSpeed::Decelerate;
                vel = this->goal_v  - MAX_ACCEL;                
            }
        }        

        if (vel > tgt_lane_info[1]) {
            vel = tgt_lane_info[1] ;
            speed = TrajectoryActionSpeed::Decelerate;
        }        


        // TrajectoryAction trajectory = TrajectoryAction(speed, TrajectoryActionLaneChange::KeepLane, this->goal_lane);
        trajectory = TrajectoryAction(speed, TrajectoryActionLaneChange::KeepLane, this->goal_lane);        
        trajectory.s = this->s;
        trajectory.goal_s = this->goal_s;
        trajectory.v = vel;       
        trajectory.goal_lane = this->lane;       
        trajectory.state = state ;     
        trajectory.tgt_lane_dist = (curr_lane_info[0] + tgt_lane_info[0])/2.0;        
        trajectory.tgt_lane_vel = (curr_lane_info[1] + tgt_lane_info[1])/2.0;        
        trajectory.tgt_lane_coll = (curr_lane_info[4] + tgt_lane_info[4])/2.0;
        // return trajectory;
    }

    if (state.compare("PLCL") == 0) {
        cout << "PLCL: Ego.goal_v=" << this->goal_v << " ego_v:" << this->v << " tgt_ln_v=" << trajectory.tgt_lane_vel << " new lane:" << new_lane << "\n";
    } else {
        cout << "PLCR: Ego.goal_v=" << this->goal_v << " ego_v:" << this->v << " tgt_ln_v=" << trajectory.tgt_lane_vel << " new lane:" << new_lane << "\n";
    }

    return trajectory;    

}

TrajectoryAction Vehicle::lane_change_trajectory(string state, map<int, TrajectoryAction> predictions, vector<vector<double>> &traffic_info) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    if (new_lane < 0) {
        new_lane = 0;
    }

    if (new_lane >= NUM_LANES) {
        new_lane = NUM_LANES - 1;
    }

    vector<double> tgt_lane_info = traffic_info[new_lane];

    if (tgt_lane_info[4] == 1) {
        // possible collision
        if (state.compare("LCL") == 0) {
            cout << "Can not LCL due to risk of colllision\n";
        } else {
            cout << "Can not LCR due to risk of colllision\n";
        }

        return NULL_TRAJECTORY_ACTION;
    }

    TrajectoryActionLaneChange lc = TrajectoryActionLaneChange::KeepLane;
    if (state.compare("LCL") == 0) {
        lc = TrajectoryActionLaneChange::ChangeLeft;
    } else if (state.compare("LCR") == 0) {
        lc = TrajectoryActionLaneChange::ChangeRight;
    }
   
    TrajectoryActionSpeed speed;
    double vel;
    if (this->goal_v >= SPEED_LIMIT) {
        speed = TrajectoryActionSpeed::Decelerate;
        vel = this->goal_v - MAX_ACCEL;       
    } else {
        speed = TrajectoryActionSpeed::MaintainSpeed;
        vel = this->goal_v ;       
    }

    if (tgt_lane_info[1] < vel) {
        vel = tgt_lane_info[1] ;
        speed = TrajectoryActionSpeed::Decelerate;        
        cout << "LC Traj vel decreasing to:" << vel << "\n" ;        
    }

    TrajectoryAction trajectory = TrajectoryAction(speed, lc, new_lane);

    trajectory.s = this->s;
    trajectory.goal_s = this->goal_s;
    trajectory.v = vel;    
    trajectory.goal_lane = new_lane;    
    trajectory.state = state ;       
    trajectory.tgt_lane_dist = tgt_lane_info[0];        
    trajectory.tgt_lane_vel = tgt_lane_info[1];        
    trajectory.tgt_lane_coll = tgt_lane_info[4];


    cout << "LC Traj Gen:" << state << "\n" ;

    if (state.compare("LCL") == 0) {
        cout << "LCL: v=" << trajectory.v << " tgt_ln_v=" << trajectory.tgt_lane_vel << "new lane:" << new_lane << "\n";
    } else {
        cout << "LCR: v=" << trajectory.v << " tgt_ln_v=" << trajectory.tgt_lane_vel << "new lane:" << new_lane << "\n";
    }

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
	TrajectoryAction predictions = TrajectoryAction(TrajectoryActionSpeed::MaintainSpeed, 
          TrajectoryActionLaneChange::KeepLane, this->goal_lane);
    predictions.v = this->v;
    //predictions.s = (double) this->s + (horizon * 0.02 * this->v) ; 
    predictions.s = this->s;
    predictions.goal_s = this->goal_s;

    return predictions;
}