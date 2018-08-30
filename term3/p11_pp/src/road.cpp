#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "include/road.h"
#include "include/vehicle.h"
#include "include/spline.h"
#include "include/utils.h"

using json = nlohmann::json;

void printPts(vector<double> x, vector<double> y)
{
  cout << "X:Y"
       << "\n";
  for (int i = 0; i < x.size(); i++)
  {
    cout << x[i] << ":" << y[i] << "\n";
  }
}

void printVhcl(Vehicle v)
{
  cout << "Vehicle[" << v.id << "] s=" << v.s << " goal_s=" << v.goal_s << "v=" << v.v << " lane=" << v.lane << " goal_d=" << v.goal_d << "\n";
}

/**
 * Initializes Road
 */
Road::Road(vector<double> wp_x, vector<double> wp_y, vector<double> wp_s,
           vector<double> wp_dx, vector<double> wp_dy) 
: lane_traffic(3, vector<double>(5)), lane_traffic_curr(3, vector<double>(2))
{
  this->_wp_x = wp_x;
  this->_wp_y = wp_y;
  this->_wp_s = wp_s;
  this->_wp_dx = wp_dx;
  this->_wp_dy = wp_dy;


  double lane_nearest_s = TrajectoryAction::DIST_LIMIT;
  double lane_nearest_s_curr = TrajectoryAction::DIST_LIMIT;
  double lane_nearest_v = TrajectoryAction::VEL_LIMIT;

  double lane_behind_nearest_s = TrajectoryAction::DIST_LIMIT;
  double lane_behind_nearest_s_curr = TrajectoryAction::DIST_LIMIT;  
  double lane_behind_nearest_v = TrajectoryAction::VEL_LIMIT;
  double vehicle_on_tgt_loc = 0; // 0 - false, 1 - true

  for (int i = 0; i < Road::NUM_LANES; i++)
  {

    // cout << "initialized I = " << i << "\n";

    vector<double> *lane_info = &lane_traffic[i] ;
    lane_info->at(0) = lane_nearest_s;
    lane_info->at(1) = lane_nearest_v;
    lane_info->at(2) = lane_behind_nearest_s;
    lane_info->at(3) = lane_behind_nearest_v;
    lane_info->at(4) = vehicle_on_tgt_loc;

    vector<double> *lane_info_curr = &lane_traffic_curr[i] ;
    lane_info_curr->at(0) = lane_nearest_s_curr;
    lane_info_curr->at(1) = lane_behind_nearest_s_curr;
  }

}

Road::~Road()
{
}

void updateOtherVehicles(map<int, Vehicle> &vehicles, const json &sensor_fusion,
                         double rd_orientation, int horizon)
{

  // reset the flags
  for (auto& obj: vehicles) {
    obj.second.updated = false; 
  }

  for (unsigned int i = 0; i < sensor_fusion.size(); i++)
  {
    int id = sensor_fusion[i][0];
    double x = sensor_fusion[i][1];
    double y = sensor_fusion[i][2];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double s = sensor_fusion[i][5];
    float d = sensor_fusion[i][6];
    double yaw = atan2(vy, vx);
    double yaw_rel_lane = yaw - rd_orientation;

    // cout << "id " << id << " x " << x << " y " << y << " vx " << vx << " vy " << vy << " s " << s << " d " << d << "\n";

    if ((d < 0.0) || (d > 12.0))
    {
      // cout << "Invalid D value skipping.\n" ;
      continue;
    }

    //Vehicle newV(x, y, vx, vy, s, d, yaw, yaw_rel_lane);
    std::map<int, Vehicle>::iterator it = vehicles.find(id);
    if (it == vehicles.end())
    {
      Vehicle *newV = new Vehicle(id, x, y, vx, vy, s, d, yaw, yaw_rel_lane);
      //cout << "v.lane = " << newV.lane << "\n" ;

      if (horizon > 1)
      {
        newV->updateGoal(horizon);
      }

      vehicles.insert(std::pair<int, Vehicle>(id, *newV));

      // printf("Not found %d\n", id);
    }
    else
    {

      // cout << "update d = " << d << "\n" ;
      //Vehicle v = it->second;
      it->second.update(x, y, vx, vy, s, d, yaw, yaw_rel_lane);
      it->second.updated = true;
      // cout << "v.lane = " << v.lane << "\n" ;

      if (horizon > 1)
      {
        it->second.updateGoal(horizon);
      }

      // printf("Found %d\n", id) ;
    }
  }

  for (auto& obj: vehicles) {
    if (!obj.second.updated) {
      int id = obj.first ;
      delete &(obj.second) ;
      vehicles.erase (id); 
    }
  }

}

/**
 * Determine 
 * - target/goal lane
 * - whether to accelerate, decelerate or stay at same speed.
 * 
 * */

TrajectoryAction Road::choose_ego_next_state(double ego_s, double ego_d, int frame,
                                             map<int, Vehicle> &vehicles, Vehicle &ego)
{

  // cout << "Choosing next state for Ego in Road\n" ;

  map<int, TrajectoryAction> predictions;

  map<int, Vehicle>::iterator it = vehicles.begin();
  while (it != vehicles.end())
  {
    int v_id = it->first;
    TrajectoryAction preds = it->second.generate_predictions(frame);
    predictions[v_id] = preds;
    it++;
  }

  // cout << "Getting traffic info\n" ;
  vector<vector<double>> traffic_info = get_traffic_kinematics(vehicles, ego);
  // for (int i = 0; i < Road::NUM_LANES; i++)
  // {
  //   vector<double> lane_info = traffic_info[(double)i];
  //   cout << "Lane " << i << " fs:" << lane_info[0] << " fv:" << lane_info[1] << " bs:" << lane_info[2] << " bv:" << lane_info[3] << " coll:" << lane_info[4] << "\n";
  // }

  // cout << "Got traffic info\n" ;

  TrajectoryAction egoAction = this->ego.choose_next_state(predictions, traffic_info, frame);

  cout << "Next state: cl(k0/l1/r2):" << (int)egoAction.changeLane << " goal lane:" << egoAction.goal_lane << " Speed action(a0/d1/m2):" << (int)egoAction.speedAction << "\n";

  return egoAction;
}

void Road::update(const json &jsn)
{
  // Main car's localization Data

  //json[1] structure.
  //["x"] The car's x position in map coordinates
  //["y"] The car's y position in map coordinates
  //["s"] The car's s position in frenet coordinates
  //["d"] The car's d position in frenet coordinates
  //["yaw"] The car's yaw angle in the map
  //["speed"] The car's speed in MPH
  // Note: Return the previous list but with processed points removed, can be a
  // nice tool to show how far along the path has processed since last time.
  //["previous_path_x"] The previous list of x points previously given to the simulator
  //["previous_path_y"] The previous list of y points previously given to the simulator
  //Previous path's end s and d values
  //["end_path_s"] The previous list's last point's frenet s value
  //["end_path_d"] The previous list's last point's frenet d value

  //Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)
  //["sensor_fusion"] A 2d vector of cars and then that car's
  // [car's unique ID, car's x position in map coordinates, car's y position in map coordinates,
  //  car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates,
  //  car's d position in frenet coordinates.

  double car_x = jsn[1]["x"];
  double car_y = jsn[1]["y"];
  double car_s = jsn[1]["s"];
  double car_d = jsn[1]["d"];
  double car_yaw = jsn[1]["yaw"];
  double car_speed = jsn[1]["speed"];

  // Previous path data given to the Planner
  auto previous_path_x = jsn[1]["previous_path_x"];
  auto previous_path_y = jsn[1]["previous_path_y"];

  // cout << " prev path" << "\n" ;
  // printPts (previous_path_x, previous_path_y);

  // Previous path's end s and d values
  double end_path_s = jsn[1]["end_path_s"];
  double end_path_d = jsn[1]["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  // [ id, x, y, vx, vy, s, d]
  auto sensor_fusion = jsn[1]["sensor_fusion"];

  double rd_orientation = road_orientation(car_s, _wp_s, _wp_x, _wp_y);

  int prev_size = previous_path_x.size();

  this->ego.update(car_x, car_y, 0, 0, car_s, car_d, car_yaw, 0);
  this->ego.v = ref_vel; // in m/sec.  if miles/hour ref_vel * * 0.4470388889 = m/sec // (1609.34 meters/mile)/(3600 secs) = 0.4470388889   or (1/2.2369418519)
  this->ego.goal_v = this->ego.v;
  this->ego.id = 555;
  cout << "Ego s = " << car_s << " " << " d = " << car_d << " v=" << this->ego.v << " car_speed:" << car_speed << " lane=" << this->ego.lane << "\n";

  // Update other vehicle data
  //  if (this->vehicles.size() <= 0){
  //    addOtherVehicles(this->vehicles, sensor_fusion, rd_orientation);
  //  } else {
  //this->vehicles = updateOtherVehicles(this->vehicles, sensor_fusion, rd_orientation, prev_size);
  updateOtherVehicles(this->vehicles, sensor_fusion, rd_orientation, prev_size);
  // printVhcl(this->vehicles[0]);
  cout << this->vehicles.size() << " size of Vehicles Map \n";
  //  }

  //Initialize
  vector<double> next_x_vals_n;
  vector<double> next_y_vals_n;
  this->next_x_vals = next_x_vals_n;
  this->next_y_vals = next_y_vals_n;

  TrajectoryActionSpeed egoAccel = TrajectoryActionSpeed::MaintainSpeed;

  //
  // go around w/o jerk, use spline
  //
  if (prev_size > 0)
  {
    car_s = end_path_s;
    car_d = end_path_d;

    this->ego.updateGoal(car_s, car_d, prev_size);
    this->ego.goal_v = ref_vel;
    this->ego.v = ref_vel;

    cout << "Ego trajectory in lane:" << this->ego.lane << "\n";

    if (lane != this->ego.lane)
    {
      // Still in Lane changing process..
      // if (ref_vel < Road::SPEED_LIMIT)
      // {
        //ref_vel -= Road::MAX_ACCEL; 
      // }
      cout << "Ego is in lane:" << (int)car_d / 4 << " d=" << car_d << "\n";
    }
    else
    {
      TrajectoryAction ta = this->choose_ego_next_state(car_s, car_d, prev_size, this->vehicles, this->ego);

      if (ta.speedAction == TrajectoryActionSpeed::Decelerate)
      {
        //ref_vel -= Road::MAX_ACCEL; 
        egoAccel = TrajectoryActionSpeed::Decelerate;
      }
      else if ((ta.speedAction == TrajectoryActionSpeed::Accelerate) && (ref_vel < SPEED_LIMIT))
      {
        //ref_vel += Road::MAX_ACCEL; 
        egoAccel = TrajectoryActionSpeed::Accelerate;
      }

      if (ta.changeLane == TrajectoryActionLaneChange::ChangeLeft)
      {
        if (lane > 0)
        {
          lane--;
          cout << " Changing lane to " << lane << "\n" ;
        }
      }
      else if (ta.changeLane == TrajectoryActionLaneChange::ChangeRight)
      {
        if (lane < (Road::NUM_LANES - 1))
        {
          lane++;
        }
      }

      this->ego.state = ta.state;
      cout << "Ego state changed to " << this->ego.state << " lane " << lane << "\n";
    }
  }
  else
  {
    if (ref_vel < Road::SPEED_LIMIT)
    {
      //ref_vel += Road::MAX_ACCEL; 
        egoAccel = TrajectoryActionSpeed::Accelerate;      
    }
  }

  vector<double> ptsx;
  vector<double> ptsy;

  // cout << " ptsxy empty" << "\n" ;
  // printPts (ptsx, ptsy);

  // ref states
  double car_x_calc = car_x;
  double car_y_calc = car_y;
  double ref_yaw = deg2rad(car_yaw);

  if (prev_size < 2)
  {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsy.push_back(prev_car_y);

    ptsx.push_back(car_x_calc);
    ptsy.push_back(car_y_calc);

    // cout << " ptsxy prev < 2 " << " prev sz " << prev_size << " pts""\n" ;
    // printPts (ptsx, ptsy);
  }
  else
  {
    car_x_calc = previous_path_x[prev_size - 1];
    car_y_calc = previous_path_y[prev_size - 1];

    double prev_car_x = previous_path_x[prev_size - 2];
    double prev_car_y = previous_path_y[prev_size - 2];

    ref_yaw = atan2(car_y_calc - prev_car_y, car_x_calc - prev_car_x);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x_calc);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y_calc);

    // cout << " ptsxy prev >=  2"  << " prev sz " << prev_size << " pts""\n" ;
    // printPts (ptsx, ptsy);
  }

//  MapUtil mu;

  // add 3 more forward way points. Use Frenet.
  
  // vector<double> next_wp = mu.getXY_spline(car_s + 30, (2 + 4 * lane));  
  vector<double> next_wp = getXY(car_s + 30, (2 + 4 * lane),
                                 this->_wp_s, this->_wp_x, this->_wp_y);
  ptsx.push_back(next_wp[0]);
  ptsy.push_back(next_wp[1]);

  // vector<double> next_wp1a = mu.getXY_spline(car_s + 45, (2 + 4 * lane));  
  // ptsx.push_back(next_wp1a[0]);
  // ptsy.push_back(next_wp1a[1]);

  // vector<double> next_wp2 = mu.getXY_spline(car_s + 60, (2 + 4 * lane));
  vector<double> next_wp2 = getXY(car_s + 60, (2 + 4 * lane),
                                  this->_wp_s, this->_wp_x, this->_wp_y);
  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp2[1]);

  // vector<double> next_wp3a = mu.getXY_spline(car_s + 75, (2 + 4 * lane));  
  // ptsx.push_back(next_wp3a[0]);
  // ptsy.push_back(next_wp3a[1]);

  // vector<double> next_wp3 = mu.getXY_spline(car_s + 90, (2 + 4 * lane));
  vector<double> next_wp3 = getXY(car_s + 90, (2 + 4 * lane),
                                  this->_wp_s, this->_wp_x, this->_wp_y);
  ptsx.push_back(next_wp3[0]);
  ptsy.push_back(next_wp3[1]);

  // cout << " ptsxy before car reference" << "\n" ;
  // printPts (ptsx, ptsy);

  // Shift the points to car reference
  for (int i = 0; i < ptsx.size(); i++)
  {
    double shift_x = ptsx[i] - car_x_calc;
    double shift_y = ptsy[i] - car_y_calc;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // cout << " ptsxy after car reference" << "\n" ;
  // printPts (ptsx, ptsy);

  //previous path points
  for (int i = 0; i < previous_path_x.size(); i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // create spline, reference to the Car co-ordinate
  tk::spline spln;

  // printPts (ptsx, ptsy);

  spln.set_points(ptsx, ptsy); //ptsx and ptsy now have co-ordinate relative to the car.

  double target_x = 30.0;
  double target_y = spln(target_x);
  double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
  // double N = (target_dist / (0.02 * ref_vel)); //( x miles/hr / 2.24) = mts per sec  // (0.02 * ref_vel / 2.24)
  // double delta = (target_x) / N;
  //printf("N =  %f\nDelta = %f\n", N, delta);

  // vector<double> carXY;
  // vector<double> carXYShifted;

  // double curr_car_s = car_s ;
  // double delta = (0.02 * ref_vel);

  double x_ref = 0;
  double y_ref = 0;

  double vel_delta = 0.0;

  if (egoAccel == TrajectoryActionSpeed::Accelerate) 
  {
    vel_delta = MAX_ACCEL;
  } else if (egoAccel == TrajectoryActionSpeed::Accelerate) {
    vel_delta = -MAX_ACCEL;
  }

  // fill up the rest of the path
  for (int i = 1; i <= 50 - previous_path_x.size(); i++)
  {
    // x_ref += delta;
    // curr_car_s += delta;
    // carXY = getXY(curr_car_s, (2 + 4 * lane), this->_wp_s, this->_wp_x, this->_wp_y);
    // carXY = mu.getXY_spline(curr_car_s, (2 + 4 * lane));
    // double shift_x = carXY[0] - car_x_calc;
    // double shift_y = carXY[1] - car_y_calc;
    // x_ref = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));

    y_ref = spln(x_ref); //y_point;

    ref_vel += vel_delta;
    double N = (target_dist / (0.02 * ref_vel)); //( x miles/hr / 2.24) = mts per sec  // (0.02 * ref_vel / 2.24)
    double delta = (target_x) / N;
    x_ref += delta;    

    // y_ref = spln(carXY[0]) ;
    // next_x_vals.push_back(carXY[0]);
    // next_x_vals.push_back(x_ref);
    // next_y_vals.push_back(y_ref);


    // // rotate back to normal/global co-ordinates
    double x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    double y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
    x_point += car_x_calc;
    y_point += car_y_calc;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);

    // cout << " next x, y " << "\n" ;
    // printPts (next_x_vals, next_y_vals);
  }
}

vector<vector<double>> Road::get_traffic_kinematics(map<int, Vehicle> vehicles, Vehicle ego)
{

  cout << "In Road::get_traffic_kinematics\n";

  double ego_goal_s = ego.goal_s;
  double ego_s = ego.s;
  double ego_v = ego.v;
  int ego_lane = ego.goal_lane;
  int horizon = ego.goal_horizon;
  int ego_id = ego.id;

  int v_id;
  int vhcl_lane;
  double vhcl_s;
  double vhcl_goal_s;
  double vhcl_v;

  // cout << "Road::NUM_LANES " << Road::NUM_LANES << "\n" ;


  // cout << "In Road::get_traffic_kinematics, initialized the map\n" ;

  cout << "Ego "
       << " lane=" << ego_lane << " s=" << ego.s << " goal_s=" << ego_goal_s << " v=" << ego_v << "\n";

  vector<double> *lane_info;
  vector<double> *lane_info_curr;  

  double lane_nearest_s = TrajectoryAction::DIST_LIMIT;
  double lane_nearest_s_curr = TrajectoryAction::DIST_LIMIT;
  double lane_nearest_v = TrajectoryAction::VEL_LIMIT;

  double lane_behind_nearest_s = TrajectoryAction::DIST_LIMIT;
  double lane_behind_nearest_s_curr = TrajectoryAction::DIST_LIMIT;  
  double lane_behind_nearest_v = TrajectoryAction::VEL_LIMIT;
  double vehicle_on_tgt_loc = 0; // 0 - false, 1 - true

  for (int i = 0; i < Road::NUM_LANES; i++)
  {

    // cout << "initialized I = " << i << "\n";

    vector<double> *lane_info = &lane_traffic[i] ;
    lane_info->at(0) = lane_nearest_s;
    lane_info->at(1) = lane_nearest_v;
    lane_info->at(2) = lane_behind_nearest_s;
    lane_info->at(3) = lane_behind_nearest_v;
    lane_info->at(4) = vehicle_on_tgt_loc;

    vector<double> *lane_info_curr = &lane_traffic_curr[i] ;
    lane_info_curr->at(0) = lane_nearest_s_curr;
    lane_info_curr->at(1) = lane_behind_nearest_s_curr;
  }



  map<int, Vehicle>::iterator it = vehicles.begin();
  while (it != vehicles.end())
  {
    v_id = it->first;

    // cout << "Vehicle ID:" << v_id << "\n" ;

    if (v_id != ego_id)
    {
      Vehicle vhcl = it->second;
      vhcl_s = vhcl.s;
      vhcl_goal_s = vhcl.goal_s;
      vhcl_v = vhcl.v;
      vhcl_lane = vhcl.lane;
      // cout << "Vhcl[" << v_id << "]" << " s=" << vhcl.s << " goal_s=" << vhcl_goal_s << " v=" << vhcl_v << " lane=" << vhcl_lane << "\n" ;

      lane_info = &lane_traffic[vhcl_lane];
      lane_info_curr = &lane_traffic_curr[vhcl_lane];
      // cout << "lane_info size " << lane_info.size() << "\n" ;

      // Predicted distcances
      if (vhcl_goal_s > ego_goal_s)
      {
        // Vehicle is infront        
        // cout << "veh_S > ego_goal_s" << "\n" ;
        if (lane_info->at(0) > (vhcl_goal_s - ego_goal_s))
        {
          // cout << "front, new closest " << vhcl.id << " dist " << (vhcl_goal_s - ego_goal_s) << " vel " << vhcl_v << "\n" ;
          lane_info->at(0) = vhcl_goal_s - ego_goal_s;
          lane_info->at(1) = vhcl_v;

          // Less than 2 sec time gap from front vehicles in current and target lanes, from predicted positions.
          // Less than  minimum meters gap from front vehicles in current and target lanes, from predicted positions.
          if (abs(lane_info->at(0)) <= Road::PREF_BUFFER)
          {
            cout << "possible collision if in lane " << vhcl_lane << "\n";
            lane_info->at(4) = (double)1; // possible collision
          }          

          double time_gap = abs(lane_info->at(0)/ego_v) ;
          if (time_gap < Road::MIN_TIME_GAP){
            cout << "possible collision if in lane due to less tg " << vhcl_lane << "\n";
            lane_info->at(4) = (double)1; // possible collision
          }
        }

        if (ego_v > vhcl_v) 
        {
          double v_diff = ego_v - vhcl_v;
          double time_gap = abs((vhcl_goal_s - ego_goal_s)/v_diff) ;
          if (time_gap < Road::MIN_TIME_GAP){
            cout << "possible collision if in lane due to less tg and ego speed diff " << vhcl_lane << "\n";            
            lane_info->at(4) = (double)1; // possible collision
          }
        }
      }
      else
      {
        // Vehicle is behind
        // cout << "veh_S <= ego_goal_s, vhcl_lane = " << vhcl_lane << " lane_traffic_sz =" << lane_traffic.size() << "\n" ;
        if (lane_info->at(2) > (ego_goal_s - vhcl_goal_s))
        {
          // cout << "behind, new closest" << vhcl.id << " dist " << (ego_goal_s - vhcl_goal_s) << " vel " << vhcl_v << "\n" ;
          lane_info->at(2) = ego_goal_s - vhcl_goal_s;
          lane_info->at(3) = vhcl_v;

          // Less than 2 sec time gap from front vehicles in current and target lanes, from predicted positions.
          // Less than  minimum meters gap from front vehicles in current and target lanes, from predicted positions.
          if ((abs(lane_info->at(2)) <= Road::PREF_BUFFER) && (ego_lane != vhcl_lane))
          {
            cout << "possible collision if lane change " << vhcl_lane << "\n";
            lane_info->at(4) = (double)1; // possible collision
          }          

          double time_gap = abs(lane_info->at(0)/ego_v) ;
          if ((time_gap < Road::MIN_TIME_GAP) && (ego_lane != vhcl_lane))
          { 
            cout << "possible collision if lane change TG with bv " << vhcl_lane << "\n";            
            lane_info->at(4) = (double)1; // possible collision
          }          
        }
      }

      //Curr distance
      if (vhcl_s > ego_s)
      {
        // Vehicle is infront
        // cout << "veh_S > ego_goal_s" << "\n" ;
        if (lane_info_curr->at(0) > (vhcl_s - ego_s))
        {
          // cout << "front, new closest " << vhcl.id << " dist " << (vhcl_goal_s - ego_goal_s) << " vel " << vhcl_v << "\n" ;
          lane_info_curr->at(0) = vhcl_s - ego_s;

          // Less than 2 sec time gap from front vehicles in current and target lanes, from predicted positions.
          // Less than  minimum meters gap from front vehicles in current and target lanes, from predicted positions.
          if (abs(lane_info_curr->at(0)) <= Road::PREF_BUFFER)
          {
            cout << "possible collision if in lane " << vhcl_lane << "\n";
            lane_info->at(4) = (double)1; // possible collision
          }          

          double time_gap = abs(lane_info_curr->at(0)/ego_v) ;
          if (time_gap < Road::MIN_TIME_GAP){
            cout << "possible collision if in lane due to less tg fv curr s " << vhcl_lane << "\n";            
            lane_info->at(4) = (double)1; // possible collision
          }
        }

        if (ego_v > vhcl_v) 
        {
          double v_diff = ego_v - vhcl_v;
          double time_gap = abs((vhcl_s - ego_s)/v_diff) ;
          if (time_gap < Road::MIN_TIME_GAP){
            cout << "possible collision if in lane due to less tg and ego speed diff curr s " << vhcl_lane << "\n";                      
            lane_info->at(4) = (double)1; // possible collision
          }
        }        
      }
      else
      {
        // Vehicle is behind
        // cout << "veh_S <= ego_goal_s, vhcl_lane = " << vhcl_lane << " lane_traffic_sz =" << lane_traffic.size() << "\n" ;
        if (lane_info_curr->at(1) > (ego_s - vhcl_s))
        {
          // cout << "behind, new closest" << vhcl.id << " dist " << (ego_goal_s - vhcl_goal_s) << " vel " << vhcl_v << "\n" ;
          lane_info_curr->at(1) = ego_s - vhcl_s;

          // Less than 2 sec time gap from front vehicles in current and target lanes, from predicted positions.
          // Less than  minimum meters gap from front vehicles in current and target lanes, from predicted positions.
          if ((abs(lane_info_curr->at(1)) <= Road::PREF_BUFFER) && (ego_lane != vhcl_lane))
          {
            cout << "possible collision if lane change " << vhcl_lane << "\n";
            lane_info->at(4) = (double)1; // possible collision
          }          

          double time_gap = abs(lane_info_curr->at(1)/ego_v) ;
          if ((time_gap < Road::MIN_TIME_GAP) && (ego_lane != vhcl_lane))
          {
            cout << "possible collision if lane change TG with bv curr s " << vhcl_lane << "\n";                       
            lane_info->at(4) = (double)1; // possible collision
          }   
        }
      }
    }
    else
    {
      cout << "Veh == Ego \n";
    }

    it++;
  }

  for (int i = 0; i < Road::NUM_LANES; i++)
  {
    //cout << "Lane " << i << " fs:" <<  lane_traffic[i][0] << " fv:" <<  lane_traffic[i][1] << " bs:" <<  lane_traffic[i][2] << " bv:" <<  lane_traffic[i][3] << " coll:" << la lane_traffic[i]ne_info[4] << "\n";
    printf("Lane %01d ", i);
    printf("g-fvs %06.2f ", lane_traffic[i][0]);
    printf("g-fvv %06.2f ", lane_traffic[i][1]);
    printf("g-bvs %06.2f ", lane_traffic[i][2]);
    printf("g-bvv %06.2f ", lane_traffic[i][3]);
    printf("g-lc  %06.2f ", lane_traffic[i][4]);    
    printf("c-fvs %06.2f ", lane_traffic_curr[i][0]);    
    printf("c-bvs %06.2f ", lane_traffic_curr[i][1]);   
    printf("\n");
  }

  // cout << "Returning from  Road::get_traffic_kinematics\n" ;
  return lane_traffic;
}
