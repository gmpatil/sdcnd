/**
 * Initializes Vehicle
 */

#include "include/trajectory_action.h"


TrajectoryAction::TrajectoryAction() { }

TrajectoryAction::TrajectoryAction(bool decel, bool cl, int ln){
  this->decelerate = decel;
  this->change_lane = cl;
  this->lane = ln;  
}

TrajectoryAction::~TrajectoryAction() { }
