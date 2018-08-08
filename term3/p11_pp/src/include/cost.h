#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

float calculate_cost(const Vehicle &vehicle, const map<int, vector<TrajectoryAction>> &predictions, const vector<TrajectoryAction> &trajectory);

float goal_distance_cost(const Vehicle &vehicle, const vector<TrajectoryAction> &trajectory, const map<int, vector<TrajectoryAction>> &predictions, map<string, float> &data);

float inefficiency_cost(const Vehicle &vehicle, const vector<TrajectoryAction> &trajectory, const map<int, vector<TrajectoryAction>> &predictions, map<string, float> &data);

float lane_speed(const map<int, vector<TrajectoryAction>> &predictions, int lane);

map<string, float> get_helper_data(const Vehicle &vehicle, const vector<TrajectoryAction> &trajectory, const map<int, vector<TrajectoryAction>> &predictions);

#endif