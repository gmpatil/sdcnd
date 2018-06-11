#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

using namespace std;

static const double TRACKLENGTH = 6945.554;
  
// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}

double deg2rad(double x);

double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2) ;

/*
 Returns the maps' index, which is closest way point for global (x, y) co-ordinates.
 */

int closestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) ;

int nextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
        const vector<double> &maps_y) ;

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates

vector<double> getFrenet(double x, double y, double theta,
        const vector<double> &maps_x, const vector<double> &maps_y) ;

// Transform from Frenet s,d coordinates to Cartesian x,y

vector<double> getXY(double s, double d, const vector<double> &maps_s,
        const vector<double> &maps_x, const vector<double> &maps_y);

double road_orientation(double s, const vector<double> &maps_s,
        const vector<double> &maps_x, const vector<double> &maps_y);

#endif /* UTILS_H */
