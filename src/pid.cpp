#include "pid.h"

PIDController::PIDController(float kp, float ki, float kd, float out_min, float out_max)
: kp_(kp), ki_(ki), kd_(kd), integral_(0.0f), prev_error_(0.0f),
  out_min_(out_min), out_max_(out_max) {}

float PIDController::update(float error, float dt_seconds)
{
  integral_ += error * dt_seconds;

  float derivative = (dt_seconds > 0.0f) ? (error - prev_error_) / dt_seconds : 0.0f;
  float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

  // Anti-windup via integral clamping
  if (output > out_max_) {
    output = out_max_;
    if (ki_ != 0.0f && error > 0.0f) {
      integral_ -= error * dt_seconds;
    }
  } else if (output < out_min_) {
    output = out_min_;
    if (ki_ != 0.0f && error < 0.0f) {
      integral_ -= error * dt_seconds;
    }
  }

  prev_error_ = error;
  return output;
}

void PIDController::reset()
{
  integral_ = 0.0f;
  prev_error_ = 0.0f;
}

void PIDController::set_gains(float kp, float ki, float kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::set_output_limits(float min, float max)
{
  out_min_ = min;
  out_max_ = max;
}

WheelController::WheelController()
: pid_(1.2f, 2.0f, 0.0f),
  wheel_radius_meters_(0.0598f),
  counts_per_revolution_(1954),
  ema_alpha_(0.15f),
  zero_clamp_meters_per_sec_(0.005f),
  filtered_speed_meters_per_sec_(0.0f),
  target_velocity_(0.0f),
  min_speed_(0.0f),
  max_speed_(25.08361f) {}

void WheelController::setup(int forward_pin, int reverse_pin)
{
  forward_pin_ = forward_pin;
  reverse_pin_ = reverse_pin;
  pinMode(forward_pin_, OUTPUT);
  pinMode(reverse_pin_, OUTPUT);
}

void WheelController::update_speed(long delta_counts, float dt_seconds)
{
  float meters_per_pulse = (2.0f * PI * wheel_radius_meters_) / (float)counts_per_revolution_;
  float speed_meters_per_sec_raw = ((float)delta_counts / dt_seconds) * meters_per_pulse;

  // Apply EMA filter
  filtered_speed_meters_per_sec_ =
    (1.0f - ema_alpha_) * filtered_speed_meters_per_sec_ +
    ema_alpha_ *
    speed_meters_per_sec_raw;

  // Apply zero clamp
  if (fabs(filtered_speed_meters_per_sec_) < zero_clamp_meters_per_sec_) {
    filtered_speed_meters_per_sec_ = 0.0f;
  }
}

void WheelController::set_target_velocity(float target_velocity)
{
  // Clamp target velocity to limits
  target_velocity_ = constrain(target_velocity, -max_speed_, max_speed_);
}

void WheelController::apply_control(float current_velocity, float dt_seconds)
{
  float error = target_velocity_ - current_velocity;
  float output = pid_.update(error, dt_seconds);

  int pwm = abs((int)output);
  if (pwm > 255) {pwm = 255;}

  apply_pwm(output >= 0 ? pwm : -pwm);
}

void WheelController::apply_pwm(int pwm)
{
  if (pwm > 0) {
    analogWrite(forward_pin_, pwm);
    analogWrite(reverse_pin_, 0);
  } else if (pwm < 0) {
    analogWrite(forward_pin_, 0);
    analogWrite(reverse_pin_, -pwm);
  } else {
    analogWrite(forward_pin_, 0);
    analogWrite(reverse_pin_, 0);
  }
}

void WheelController::set_pid_gains(float kp, float ki, float kd)
{
  pid_.set_gains(kp, ki, kd);
}

void WheelController::set_wheel_parameters(float wheel_radius_meters, int counts_per_rev)
{
  wheel_radius_meters_ = wheel_radius_meters;
  counts_per_revolution_ = counts_per_rev;
}

void WheelController::set_speed_filtering(float ema_alpha, float zero_clamp)
{
  ema_alpha_ = ema_alpha;
  zero_clamp_meters_per_sec_ = zero_clamp;
}

void WheelController::set_speed_limits(float min_speed, float max_speed)
{
  min_speed_ = min_speed;
  max_speed_ = max_speed;
}
