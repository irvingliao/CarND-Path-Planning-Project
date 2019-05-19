#include "planner.h"
#include "algorithm"
#include "helpers.h"

Planner::Planner(vector<double> map_waypoints_x,
                         vector<double> map_waypoints_y,
                         vector<double> map_waypoints_s,
                         vector<double> map_waypoints_dx,
                         vector<double> map_waypoints_dy) {
  map_waypoints_x_ = map_waypoints_x;
  map_waypoints_y_ = map_waypoints_y;
  map_waypoints_s_ = map_waypoints_s;
  map_waypoints_dx_ = map_waypoints_dx;
  map_waypoints_dy_ = map_waypoints_dy;

}

void Planner::updateSensorFusion(json data) {
  // initialize ego vehicle
  ego = Vehicle(data, true);
  ego.lane = lane;

  // Reset
  car_front = ego;
  car_front_l = ego;
  car_front_r = ego;
  car_rear_l = ego;
  car_rear_r = ego;

  // Previous path data given to the Planner
  previous_path_x = data[1]["previous_path_x"];
  previous_path_y = data[1]["previous_path_y"];
  // Previous path's end s and d values
  end_path_s = data[1]["end_path_s"];
  end_path_d = data[1]["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  json sensor_fusion = data[1]["sensor_fusion"];
  int prev_size = previous_path_x.size();

  for (json sensor_fusion_vehicle : sensor_fusion) {
    Vehicle car = Vehicle(sensor_fusion_vehicle, false);
    checkSurrounding(car, prev_size, DELTA_T);
  }
}

void Planner::checkSurrounding(Vehicle &car, int prev_size, double dt) {
  // If the car is not in the range that we'll need to consider.
  if (car.lane < 0) {
    return;
  }

  double check_car_s = car.s;
  // check_car_s += static_cast<double>(prev_size * dt * car.speed);

  int check_lane = car.lane - lane;
  double distance = check_car_s - ego.s;

  double closest_dist = 99999;
  if (check_lane == 0) {
    // Has car in same lane and ahead.
    if (!car_front.isEgoCar) {
      closest_dist = car_front.s - ego.s;
    }
    if (distance > 0 && closest_dist > distance) {
      //std::cout << distance << "m ahead \n";
      car_front = car;
    }
  } else if (check_lane == -1) {
    // Has car on the left
    if (distance > 0) {
      if (!car_front_l.isEgoCar) {
        closest_dist = car_front_l.s - ego.s;
      }

      if (closest_dist > distance) {
        car_front_l = car;
      }
    } else {
      if (!car_rear_l.isEgoCar) {
        closest_dist = car_rear_l.s - ego.s;
      }

      if (fabs(closest_dist) > fabs(distance)) {
        car_rear_l = car;
      }
    }
  } else if (check_lane == 1) {
    // Has Car on the right
    if (distance > 0) {
      if (!car_front_r.isEgoCar) {
        closest_dist = car_front_r.s - ego.s;
      }

      if (closest_dist > distance) {
        car_front_r = car;
      }
    } else {
      if (!car_rear_r.isEgoCar) {
        closest_dist = car_rear_r.s - ego.s;
      }

      if (fabs(closest_dist) > fabs(distance)) {
        car_rear_r = car;
      }
    }
  }
}

json Planner::path() {
  ego.calculateCost(car_front, car_front_l, car_rear_l, car_front_r, car_rear_r);
  ego.decideAction(ref_velocity, MAX_SPEED_MPH, ACCELERATION, car_front);
  ref_velocity = ego.ref_vel;
  lane = ego.lane;

  vector<double> ptsx;
  vector<double> ptsy;
  int prev_size = previous_path_x.size();

  double car_s = ego.s;
  if (prev_size > 0) {
    car_s = end_path_s;
  }

  double ref_x = ego.x;
  double ref_y = ego.y;
  double ref_yaw = ego.yaw_rad;

  /*
   Generate Trajectory
   */

  if (prev_size < 2) {
    double prev_car_x = ego.x - cos(ego.yaw);
    double prev_car_y = ego.y - sin(ego.yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ego.x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(ego.y);
  } else {
    // Use the last two points.
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);

  }
  
  // Setting up target points in the future.
  double car_d = LANE_WIDTH * (0.5 + lane);
  vector<double> next_wp0 = getXY(car_s + 30, car_d,
                                  map_waypoints_s_,
                                  map_waypoints_x_,
                                  map_waypoints_y_);
  vector<double> next_wp1 = getXY(car_s + 60, car_d,
                                  map_waypoints_s_,
                                  map_waypoints_x_,
                                  map_waypoints_y_);
  vector<double> next_wp2 = getXY(car_s + 90, car_d,
                                  map_waypoints_s_,
                                  map_waypoints_x_,
                                  map_waypoints_y_);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // Making coordinates to local car coordinates.
  for (int i = 0; i < ptsx.size(); i++) {
    // shift car reference angle to 0 degree
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // create a spline
  tk::spline s;

  // set (x,y) points to the spline
  s.set_points(ptsx, ptsy);

  // define the actual (x,y) points we will use for the planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // start with the previous path points from last time
  for (int i = 0; i < prev_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  // Fill up the rest of our path planner after filling it with previous points

  for (int i = 1; i <= PATH_POINTS_TOTAL - prev_size; i++) {

    double N = target_dist / (DELTA_T*ref_velocity*MPH_TO_MS);
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);
    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
    y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  json msgJson;
  msgJson["next_x"] = next_x_vals;
  msgJson["next_y"] = next_y_vals;

  return msgJson;

}