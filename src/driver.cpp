#include "driver.h"

// Static member initialization
WheelController Driver::left_wheel_;
WheelController Driver::right_wheel_;
volatile long Driver::left_encoder_count_ = 0;
volatile long Driver::right_encoder_count_ = 0;
volatile long Driver::last_left_count_ = 0;
volatile long Driver::last_right_count_ = 0;
unsigned long Driver::last_update_time_ = 0;

Driver::Driver() {}

void Driver::setup()
{
  // Set up motor control
  pinMode(SLP, OUTPUT);
  digitalWrite(SLP, HIGH);

  // Initialize wheel controllers
  left_wheel_.setup(MOT_AIN1_FWD, MOT_AIN2_REV);
  right_wheel_.setup(MOT_BIN1_FWD, MOT_BIN2_REV);

  // Configure wheel parameters
  left_wheel_.set_wheel_parameters(wheel_radius_meters, counts_per_rev);
  right_wheel_.set_wheel_parameters(wheel_radius_meters, counts_per_rev);

  // Set up encoders
  pinMode(ENC_LEFT_A, INPUT);
  pinMode(ENC_LEFT_B, INPUT);
  pinMode(ENC_RIGHT_A, INPUT);
  pinMode(ENC_RIGHT_B, INPUT);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), left_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), right_encoder_isr, CHANGE);

  last_update_time_ = millis();
}

void Driver::cmd_vel_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  float ux = msg->linear.x;
  float w = -msg->angular.z;

  // Convert cmd_vel to wheel velocities
  // Left wheel velocity: υL = (2 * ux + ω*L)/(2*R)
  // Right wheel velocity: υR = (2 * ux - ω*L)/(2*R)
  float uL = ((2.0f * ux) + (w * wheel_base_meters)) / (2.0f * wheel_radius_meters);
  float uR = ((2.0f * ux) - (w * wheel_base_meters)) / (2.0f * wheel_radius_meters);

  left_wheel_.set_target_velocity(uL);
  right_wheel_.set_target_velocity(uR);
}

void Driver::left_encoder_isr()
{
  int left_wheel_current_state_a = digitalRead(ENC_LEFT_A);
  static bool forward_direction = true;

  if (left_wheel_current_state_a != LOW) {
    int left_wheel_current_state_b = digitalRead(ENC_LEFT_B);
    if (left_wheel_current_state_b == LOW && forward_direction) {
      forward_direction = false;       // Reverse
    } else if (left_wheel_current_state_b == HIGH && !forward_direction) {
      forward_direction = true;       // Forward
    }
  }

  if (!forward_direction) {
    left_encoder_count_++;
  } else {
    left_encoder_count_--;
  }
}

void Driver::right_encoder_isr()
{
  int right_wheel_current_state_a = digitalRead(ENC_RIGHT_A);
  static bool forward_direction = true;

  if (right_wheel_current_state_a != LOW) {
    int right_wheel_current_state_b = digitalRead(ENC_RIGHT_B);
    if (right_wheel_current_state_b == LOW && forward_direction) {
      forward_direction = false;
    } else if (right_wheel_current_state_b == HIGH && !forward_direction) {
      forward_direction = true;
    }
  }

  if (!forward_direction) {
    right_encoder_count_++;
  } else {
    right_encoder_count_--;
  }
}

void Driver::update()
{
  unsigned long now = millis();
  float dt_seconds = (now - last_update_time_) / 1000.0f;

  if (dt_seconds > 0.0f) {
    // Calculate speed from encoder counts
    long left_delta = left_encoder_count_ - last_left_count_;
    long right_delta = right_encoder_count_ - last_right_count_;

    // Update wheel speeds
    left_wheel_.update_speed(left_delta, dt_seconds);
    right_wheel_.update_speed(right_delta, dt_seconds);

    // Apply PID control
    left_wheel_.apply_control(left_wheel_.get_filtered_speed(), dt_seconds);
    right_wheel_.apply_control(right_wheel_.get_filtered_speed(), dt_seconds);

    // Update tracking variables
    last_left_count_ = left_encoder_count_;
    last_right_count_ = right_encoder_count_;
    last_update_time_ = now;
  }
}
