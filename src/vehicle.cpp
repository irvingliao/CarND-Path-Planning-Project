#include "vehicle.h"
#include "math.h"
#include "helpers.h"
#include <iostream>

const double LANE_WIDTH = 4.0;
const double MPH_TO_MS = 0.44704;
const double COST_KEEP = 1;
const double COST_LAND_FRONT = 1;
const double COST_LAND_REAR = 0.5;
const double MAX_COST = 99999;

Vehicle::Vehicle() {}

Vehicle::Vehicle(json &j, bool isEgo) {
  isEgoCar = isEgo;
  stay_lane_cost = 0;
  change_left_cost = MAX_COST;
  change_right_cost = MAX_COST;

  if (isEgo) {
    // Main car's localization Data
    x = j[1]["x"];
    y = j[1]["y"];
    s = j[1]["s"];
    d = j[1]["d"];
    yaw = j[1]["yaw"];
    yaw_rad = deg2rad(yaw);
    speed = j[1]["speed"];

  } else {
    id = j[0];
    x = j[1];
    y = j[2];
    vx = j[3];
    vy = j[4];
    s = j[5];
    d = j[6];
    yaw = 0.0;
    yaw_rad = 0.0;
    speed = sqrt(vx*vx+vy*vy);

    lane = -1;
    if (d > 0 && d < 12) {
      lane = static_cast<int>(d / LANE_WIDTH);
    }
  }
}

void Vehicle::calculateCost(Vehicle &car_front,
  Vehicle &car_front_l, Vehicle &car_rear_l,
  Vehicle &car_front_r, Vehicle &car_rear_r) {
  // Stay lane cost
  double ego_speed = speed*MPH_TO_MS;

  stay_lane_cost = 0;
  if (!car_front.isEgoCar && car_front.speed < ego_speed) {
    double dist_f = car_front.s - s;
    if (dist_f < 100) {
      // only consider cost within 70m.
      stay_lane_cost += 0.2;
    }

    if (dist_f < 30) {
      stay_lane_cost += 0.4;
    } else if (dist_f > 30 && dist_f < 50) {
      stay_lane_cost += 0.2;
    }
  }

  // Change left cost
  if (lane == 0) {
    change_left_cost = MAX_COST;
  } else if (car_front_l.isEgoCar && car_rear_l.isEgoCar) {
    // Left lane is empty
    change_left_cost = 0;
  } else {
    change_left_cost = 0;
    if (!car_front_l.isEgoCar) {
      double dist_f = car_front_l.s - s;
      if (car_front_l.speed < ego_speed && dist_f < 100) {
        change_left_cost += 0.2;
      }
      if (dist_f < 30) {
        change_left_cost += 10;
      } else if (dist_f > 30 && dist_f < 50) {
        change_left_cost += 0.3;
      }
    }

    if (!car_rear_l.isEgoCar) {
      if (car_rear_l.speed > ego_speed) {
        change_left_cost += 0.2;
      }
      double dist_r = s - car_rear_l.s;
      if (dist_r < 15) {
        change_left_cost += 10;
      }
      //std::cout << "Rear_L_Dist: " << dist_r << "\n";
    }
  }
  
  // Change Right cost
  if (lane == 2) {
    change_right_cost = MAX_COST;
  } else if (car_front_r.isEgoCar && car_rear_r.isEgoCar) {
    // Right lane is empty
    change_right_cost = 0;
  } else {
    change_right_cost = 0;
    if (!car_front_r.isEgoCar) {
      double dist_f = car_front_r.s - s;
      if (car_front_r.speed < ego_speed && dist_f < 100) {
        change_right_cost += 0.2;
      }
      if (dist_f < 30) {
        change_right_cost += 10;
      } else if (dist_f > 30 && dist_f < 50) {
        change_right_cost += 0.3;
      }
    }

    if (!car_rear_r.isEgoCar) {
      if (car_rear_r.speed > ego_speed) {
        change_right_cost += 0.2;
      }
      double dist_r = s - car_rear_r.s;
      if (dist_r < 15) {
        change_right_cost += 10;
      }
      //std::cout << "Rear_R_Dist: " << dist_r << "\n";
    }
  }

  std::map<double, int> cost_map = { {stay_lane_cost,  0},
                                {change_left_cost, 1},
                                {change_right_cost, 2} };

  std::map<double, int>::iterator cost_map_iterator;
  cost_map_iterator = cost_map.begin();
  int action = cost_map_iterator->second;

  if (action == 1 && stay_lane_cost == change_left_cost) {
    action = 0;
  }

  if (action == 2 && stay_lane_cost == change_right_cost) {
    action = 0;
  }

  std::cout << "Action: " << action << " | stay cost: " << stay_lane_cost 
            << " | left cost: " << change_left_cost  << " | right cost: " << change_right_cost << "\n";

  status = NORMAL;
  switch (action) {
  case 0:
    if (!car_front.isEgoCar) {
      double dist_f = car_front.s-s;
      if (dist_f < 30) {
        status = SLOW;
      } else if (car_front.speed < ego_speed && dist_f < 50) {
        status = SLOW;
      } else {
        status = NORMAL;
      }
    } else {
      status = NORMAL;
    }
    break;

  case 1:
    status = CL;
    break;

  case 2:
    status = CR;
    break;

  default:
    status = NORMAL;
    break;
  }
}

void Vehicle::decideAction(double vel, double max_speed, double acceleration, Vehicle &car_front) {
  ref_vel = vel;

  switch (status) {
    case NORMAL:
      if (ref_vel < max_speed) {
        if (car_front.isEgoCar) {
          ref_vel += 0.25*acceleration;
        }
        ref_vel += acceleration;
      } else {
        ref_vel = max_speed;
      }
      break;

    case SLOW:
      if (ref_vel > car_front.speed/MPH_TO_MS) {
        double factor = 1;
        if ((car_front.s - s) < 30) {
          factor += 0.25;
        }
        ref_vel -= factor*acceleration;
      } else if ((car_front.s - s) < 30) {
        ref_vel -= acceleration;
      } else {
        ref_vel += 0.5*acceleration;
      }
      break;

    case CL:
      lane--;
      break;

    case CR:
      lane++;
      break;

    default:
      std::cout << "Invalid Driving Status\n";
      break;
  }
}
