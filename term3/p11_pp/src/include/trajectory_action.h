
#ifndef TRAJECTORY_ACTION_H
#define TRAJECTORY_ACTION_H

#include <string>

using namespace std;

enum class TrajectoryActionSpeed { Accelerate, Decelerate, MaintainSpeed };  
enum class TrajectoryActionLaneChange { KeepLane, ChangeLeft, ChangeRight };  

class TrajectoryAction {  
  public:
    TrajectoryActionSpeed speedAction = TrajectoryActionSpeed::Accelerate;
    TrajectoryActionLaneChange changeLane = TrajectoryActionLaneChange::KeepLane;

    int goalLane = 0;
    double s;
    double v;
    string state;
    
    TrajectoryAction();
    
    TrajectoryAction(TrajectoryActionSpeed spdAct, TrajectoryActionLaneChange cl, int ln);
    
    virtual ~TrajectoryAction();
} ;

extern TrajectoryAction NULL_TRAJECTORY_ACTION;

#endif /* TRAJECTORY_ACTION_H */

