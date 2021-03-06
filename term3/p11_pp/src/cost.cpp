#include "include/cost.h"
#include "include/vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const float REACH_GOAL = pow(10, 5);
const float EFFICIENCY = pow(10, 5);

// float goal_distance_cost(const Vehicle & vehicle, const TrajectoryAction & trajectory, const map<int, TrajectoryAction> &predictions, map<string, float> & data) {
//   /*
//   Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
//   Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
//    */
//   float cost;
//   float distance = data["distance_to_goal"];
//   if (distance > 0) {
//     cost = 1 - 2 * exp(-(abs(2.0 * vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
//   } else {
//     cost = 1;
//   }
//   return cost;
// }

// float inefficiency_cost(const Vehicle & vehicle, const TrajectoryAction & trajectory, const map<int, TrajectoryAction> &predictions, map<string, float> & data) {
//   /*
//   Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed. 
//    */

//   float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
//   if (proposed_speed_intended < 0) {
//     proposed_speed_intended = vehicle.SPEED_LIMIT;
//   }

//   float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
//   if (proposed_speed_final < 0) {
//     proposed_speed_final = vehicle.SPEED_LIMIT;
//   }

//   float cost = (2.0 * vehicle.SPEED_LIMIT - proposed_speed_intended - proposed_speed_final) / vehicle.SPEED_LIMIT;

//   return cost;
// }

float goal_distance_cost(const Vehicle & vehicle, const TrajectoryAction & trajectory, const map<int, TrajectoryAction> &predictions, map<string, float> & data) {
  /*
  Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
  Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
   */

  float cost;

  // if ((trajectory.changeLane == TrajectoryActionLaneChange::ChangeLeft) || (trajectory.changeLane == TrajectoryActionLaneChange::ChangeRight)) {
  if ((trajectory.state.compare("LCL") == 0 ) || (trajectory.state.compare("LCR") == 0 )) {    
    cost = 1.15; // 0.10 for reducing velocity by 5m.
  } else if ((trajectory.state.compare("PLCL") == 0 ) || (trajectory.state.compare("PLCR") == 0 )) {   
    cost = 1.10;
  } else {
    cost = 1.0;
  }

  cost += trajectory.tgt_lane_coll;

  cost += (1.0/trajectory.tgt_lane_dist) ;

  return cost;
}

float inefficiency_cost(const Vehicle & vehicle, const TrajectoryAction & trajectory, const map<int, TrajectoryAction> &predictions, map<string, float> & data) {
  /*
  Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed. 
   */
  double proposed_speed = vehicle.goal_v;

  // if ((trajectory.state.compare("LCL") == 0 ) || (trajectory.state.compare("LCR") == 0 ) || (trajectory.state.compare("PLCL") == 0 ) || (trajectory.state.compare("PLCR") == 0 )) {    
    if (trajectory.tgt_lane_vel > proposed_speed){
      proposed_speed = trajectory.tgt_lane_vel;
    }
  // }

  float cost = ( (2.0 * Vehicle::SPEED_LIMIT) - proposed_speed) / Vehicle::SPEED_LIMIT;

  return cost;
}

float lane_speed(const map<int, TrajectoryAction> &predictions, int lane) {
  /*
  All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
  we can just find one vehicle in that lane.
   */
  for (map<int, TrajectoryAction>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
    int key = it->first;
    TrajectoryAction vehicle = it->second;
    if (vehicle.goal_lane == lane && key != -1) {
      return vehicle.v;
    }
  }
  //Found no vehicle in the lane
  return -1.0;
}

float calculate_cost(const Vehicle & vehicle, const map<int, TrajectoryAction> &predictions, const TrajectoryAction &trajectory) {
  /*
  Sum weighted cost functions to get total cost for trajectory.
   */
  map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
  float cost = 0.0;

  //Add additional cost functions here.
  vector < function<float(const Vehicle &, const TrajectoryAction &, const map<int, TrajectoryAction> &, map<string, float> &) >> cf_list = {goal_distance_cost, inefficiency_cost};
  vector<float> weight_list = {REACH_GOAL, EFFICIENCY};

  for (int i = 0; i < cf_list.size(); i++) {
    float new_cost = weight_list[i] * cf_list[i](vehicle, trajectory, predictions, trajectory_data);
    cost += new_cost;
  }

  return cost;

}

map<string, float> get_helper_data(const Vehicle & vehicle, const TrajectoryAction &trajectory, 
        const map<int, TrajectoryAction> &predictions) {
  /*
  Generate helper data to use in cost functions:
  indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
  final_lane: the lane of the vehicle at the end of the trajectory.
  distance_to_goal: the distance of the vehicle to the goal.

  Note that indended_lane and final_lane are both included to help differentiate between planning and executing
  a lane change in the cost functions.
   */
  map<string, float> trajectory_data;
  TrajectoryAction trajectory_last = trajectory;
  float intended_lane;

  if (trajectory_last.state.compare("PLCL") == 0) {
    intended_lane = trajectory_last.goal_lane + 1;
  } else if (trajectory_last.state.compare("PLCR") == 0) {
    intended_lane = trajectory_last.goal_lane - 1;
  } else {
    intended_lane = trajectory_last.goal_lane;
  }

  float distance_to_goal = vehicle.goal_s - trajectory_last.s;
  float final_lane = trajectory_last.goal_lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;
  return trajectory_data;
}

