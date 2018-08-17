/**
 * Initializes Vehicle
 */

#include "include/trajectory_action.h"


TrajectoryAction::TrajectoryAction() { }

TrajectoryAction::TrajectoryAction(TrajectoryActionSpeed spdAct, TrajectoryActionLaneChange cl, int ln){
  this->speedAction = spdAct;
  this->changeLane = cl;
  this->goalLane = ln;  
}

TrajectoryAction::~TrajectoryAction() { }

TrajectoryAction NULL_TRAJECTORY_ACTION;