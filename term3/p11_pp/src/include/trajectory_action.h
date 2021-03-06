
#ifndef TRAJECTORY_ACTION_H
#define TRAJECTORY_ACTION_H

#include <string>

using namespace std;

enum class TrajectoryActionSpeed { Accelerate, Decelerate, MaintainSpeed };  
enum class TrajectoryActionLaneChange { KeepLane, ChangeLeft, ChangeRight };  


class TrajectoryAction {  
  public:
    static constexpr double DIST_LIMIT = 100; 
    static constexpr double VEL_LIMIT =  20;

    TrajectoryActionSpeed speedAction = TrajectoryActionSpeed::Accelerate;
    TrajectoryActionLaneChange changeLane = TrajectoryActionLaneChange::KeepLane;

    double s;
    double v;
    string state;
    int goal_lane = 0; 
    double goal_s;
    double tgt_lane_vel;
    double tgt_lane_dist;
    double tgt_lane_coll;

    TrajectoryAction();
    
    TrajectoryAction(TrajectoryActionSpeed spdAct, TrajectoryActionLaneChange cl, int ln);
    
    virtual ~TrajectoryAction();
} ;

extern TrajectoryAction NULL_TRAJECTORY_ACTION;

#endif /* TRAJECTORY_ACTION_H */

