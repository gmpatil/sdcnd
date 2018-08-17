
#ifndef TRAJECTORY_ACTION_H
#define TRAJECTORY_ACTION_H

#include <string>

using namespace std;

enum class TrajectoryActionSpeed { Accelerate, Decelerate, MaintainSpeed };  

class TrajectoryAction {  
  public:
    TrajectoryActionSpeed speedAction = TrajectoryActionSpeed::Accelerate;
    bool changeLane = false;
    int goalLane = 0;
    double s;
    double v;
    string state;
    
    TrajectoryAction();
    
    TrajectoryAction(TrajectoryActionSpeed spdAct, bool cl, int ln);
    
    virtual ~TrajectoryAction();
} ;

extern TrajectoryAction NULL_TRAJECTORY_ACTION;

#endif /* TRAJECTORY_ACTION_H */

