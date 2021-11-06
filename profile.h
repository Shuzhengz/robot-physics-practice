#ifndef PROFILE_H_
#define PROFILE_H_

#include "cmath"

constexpr double kMaxVelocity = 1.0;
constexpr double kMaxAcceleration = 1.0;

class Profile {

public:
  struct ProfilePoint {
    double position = 0.0;
    double velocity = 0.0;
  };

  Profile(ProfilePoint initial) { initial_ = initial; }

  void SetGoal(ProfilePoint goal) { goal_ = goal; }

  const double GetTime(bool ignore);
  const ProfilePoint GetSetpoint(double time);
  double total_time_ = t1_ + t2_ + t3_;
  double t1_, t2_, t3_;
private:
  ProfilePoint initial_, goal_;
  bool triangle_;
};

#endif // PROFILE_H_
