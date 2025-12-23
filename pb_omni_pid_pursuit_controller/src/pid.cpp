// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_omni_pid_pursuit_controller/pid.hpp"

PID::PID(double dt, double max, double min, double kp, double kd, double ki)
: dt_(dt), max_(max), min_(min), kp_(kp), kd_(kd), ki_(ki), pre_error_(0), integral_(0)
{
}

double PID::calculate(double set_point, double pv)
{
  // Calculate error
  double error = set_point - pv;

  // Proportional term
  double p_out = kp_ * error;

  // Integral term
  integral_ += error * dt_;
  double i_out = ki_ * integral_;

  if (integral_ > 1) {
    integral_ = 1;
  } else if (integral_ < -1) {
    integral_ = -1;
  }

  // Derivative term
  double derivative = (error - pre_error_) / dt_;
  double d_out = kd_ * derivative;

  // Calculate total output
  double output = p_out + i_out + d_out;

  // Restrict to max/min
  if (output > max_)
    output = max_;
  else if (output < min_)
    output = min_;

  // Save error to previous error
  pre_error_ = error;

  return output;
}

void PID::setSumError(double sum_error) { integral_ = sum_error; }

PID::~PID() {}
