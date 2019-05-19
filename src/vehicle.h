#ifndef VEHICLE_H
#define VEHICLE_H

#include "json.hpp"
#include <vector>

using json = nlohmann::json;

/**
 * @brief A class that holds the information about a given vehicle.
 *
 * Assumptions:
 * - All lanes are same width
 * - speed calculation can be derived from vx and vy (simply implemented)
 *
 */
class Vehicle {
 public:
  Vehicle();
  Vehicle(json &j, bool isEgo);

  void decideAction(double vel, double max_speed, double acceleration, Vehicle &car_front);

  void calculateCost(Vehicle &car_front,
    Vehicle &car_front_l, Vehicle &car_rear_l,
    Vehicle &car_front_r, Vehicle &car_rear_r);

  double s;
  double d;
  double vx;
  double vy;
  double x;
  double y;
  double yaw;      // yaw in degrees
  double yaw_rad;  // yaw in radians
  int id;

  int lane;        // Is calculated by dividing d through lane width
  double speed;    // Calculated from vx and vy
  bool isEgoCar;

  // Car surround status
  double stay_lane_cost;
  double change_left_cost;
  double change_right_cost;

  /*
    Car status:
    Normal
    Slow
    CL: Change to left lane
    CR: Change to Right lane
    */
  enum DriveStatus {
    NORMAL = 1,
    SLOW = 2,
    CL = 3,
    CR = 4
  };
  DriveStatus status;

  double ref_vel;
};

#endif /* VEHICLE_H */