
#ifndef TRAJECTORY_ACTION_H
#define TRAJECTORY_ACTION_H

#include <string>

using namespace std;

class TrajectoryAction {  
  public:
    bool decelerate = false;
    bool change_lane = false;
    int lane = 0;
    double s;
    string state;
    double v;
    
    TrajectoryAction();
    
    TrajectoryAction(bool decel, bool cl, int ln);
    
    virtual ~TrajectoryAction();
} ;

#endif /* TRAJECTORY_ACTION_H */

