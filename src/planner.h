#ifndef PLANNER_H
#define PLANNER_H


#include <math.h>
#include <iostream>
#include <vector>
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"

using std::vector;
using namespace std;

const double LANE_WIDTH = 4.0;
const double MAX_SPEED_MPH = 49.5;
const double DELTA_T = 0.02;
const double MPH_TO_MS = 0.44704;
const int PATH_POINTS_TOTAL = 50;
const double SAFE_DISTANCE = 50;

// The max s value before wrapping around the track back to 0
const double TRACK_LENGTH_M = 6945.554;
const double NUMBER_OF_LANES = 3;

const double ACCELERATION = 0.35;

class Planner {
 public:
  Vehicle ego;

  Vehicle car_front;
  Vehicle car_front_l;
  Vehicle car_front_r;
  Vehicle car_rear_r;
  Vehicle car_rear_l;

  int lane = 1;
  double ref_velocity = 0;
  double closest_car_ahead_vel = 49.5;

      json previous_path_x;
  json previous_path_y;
  // Previous path's end s and d values
  double end_path_s = 0.0;
  double end_path_d = 0.0;

  vector<double> map_waypoints_x_;
  vector<double> map_waypoints_y_;
  vector<double> map_waypoints_s_;
  vector<double> map_waypoints_dx_;
  vector<double> map_waypoints_dy_;

  Planner(vector<double> map_waypoints_x, vector<double> map_waypoints_y,
        vector<double> map_waypoints_s, vector<double> map_waypoints_dx,
        vector<double> map_waypoints_dy);

  void updateSensorFusion(json data);
  json path();
  void checkSurrounding(Vehicle &car, int prev_size, double dt);
};

#endif /* PLANNER_H */