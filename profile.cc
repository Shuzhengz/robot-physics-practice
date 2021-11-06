#include "profile.h"
#include <iostream>

const double Profile::GetTime(bool ignore) {
  double delta_x = goal_.position - initial_.position;
  double discrim =
      std::pow(initial_.velocity, 2) + (2 * kMaxAcceleration * delta_x);
  double max_v = std::sqrt((kMaxAcceleration * std::pow(initial_.velocity, 2)) +
                           2 * kMaxAcceleration * goal_.position *
                               kMaxAcceleration / 2 * kMaxAcceleration);
  double total_time;
  if (ignore) {
    double full_send_time =
        (initial_.velocity + sqrt(discrim)) / kMaxAcceleration;
    //  double full_send_time = delta_x / (.5 * (goal_.velocity +
    //  initial_.velocity)); double full_send_time = (initial_.velocity +
    //  sqrt(discrim)) / kMaxAcceleration;
    return full_send_time;
  } else {
    if (max_v == kMaxVelocity) {
      // first part of trapezoid
      double accel_time = (kMaxVelocity - initial_.velocity) / kMaxAcceleration;
      double accel_dist = .5 * kMaxVelocity * accel_time;

      double vel_dist = delta_x - (2 * accel_dist);
      double vel_time = vel_dist / kMaxVelocity;

      total_time = (2 * accel_time) + vel_time;
      triangle_ = false;

      return total_time;
      // store in mem vars
      t1_ = accel_time;
      t2_ = vel_time;
      t3_ = accel_time;
    } else if (max_v < kMaxVelocity) {
      double total_time = std::sqrt((2 * delta_x) / (kMaxAcceleration + 1.0));

      triangle_ = true;
      return total_time;
      // store in mem vars
      t1_ = total_time / 2;
      t3_ = total_time / 2;
      t2_ = 0.0;
    }
  }
}

const Profile::ProfilePoint Profile::GetSetpoint(double time) {
  Profile profile(initial_);
  profile.GetTime(false);
  // velocities
  double velocity, position;
  if (!triangle_) {
    if (time < t1_) {
      velocity = initial_.velocity + (time * kMaxAcceleration); //vi + ta
      position = (initial_.velocity * time) +
                 (.5 * kMaxAcceleration * std::pow(t1_, 2)); // tvi + .5at^2
    } else if (time >= t1_ && time < (t2_ + t1_)) {
      velocity = kMaxVelocity;
      position =
          ((initial_.velocity * t1_) +
           (.5 * kMaxAcceleration * std::pow(t1_, 2))) +
          ((kMaxVelocity * (time - t1_)) + (.5 * kMaxAcceleration * std::pow((time - t1_), 2))); //(vi * t1) +

    } else if (time > (t2_ + t1_)) {
      velocity = kMaxVelocity + (time * -kMaxAcceleration);
      position =
          ((initial_.velocity * t1_) +
           (.5 * kMaxAcceleration * std::pow(t1_, 2))) +
          ((kMaxVelocity * t2_) + (.5 * kMaxAcceleration * std::pow(t2_, 2))) +
          ((kMaxVelocity * (time - (t2_ + t1_))) + (.5 * kMaxAcceleration * std::pow(time - (t2_ + t1_), 2)));
  } else {
      if (time <= t1_) {
        velocity = initial_.velocity + (time * kMaxAcceleration);
        position = (initial_.velocity * t1_) +
                   (.5 * kMaxAcceleration * std::pow(t1_, 2));
      } else if (time >= t3_) {
        double max_v =
            std::sqrt((kMaxAcceleration * std::pow(initial_.velocity, 2)) +
                      2 * kMaxAcceleration * goal_.position * kMaxAcceleration /
                          2 * kMaxAcceleration);
        velocity = max_v + (time * -kMaxAcceleration);
        position = (initial_.velocity * t3_) +
                   (.5 * kMaxAcceleration * std::pow(t3_, 2));
      }
    }

    initial_.velocity = velocity;
    initial_.position = position;
  }
  return initial_;
}
