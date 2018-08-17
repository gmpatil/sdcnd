#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

float calculate_cost(const Vehicle &vehicle, const map<int, TrajectoryAction> &predictions, const TrajectoryAction &trajectory);

float goal_distance_cost(const Vehicle &vehicle, const TrajectoryAction &trajectory, const map<int, TrajectoryAction> &predictions, map<string, float> &data);

float inefficiency_cost(const Vehicle &vehicle, const TrajectoryAction &trajectory, const map<int, TrajectoryAction> &predictions, map<string, float> &data);

float lane_speed(const map<int, TrajectoryAction> &predictions, int lane);

map<string, float> get_helper_data(const Vehicle &vehicle, const TrajectoryAction &trajectory, const map<int, TrajectoryAction> &predictions);

#endif