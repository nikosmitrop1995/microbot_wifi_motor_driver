#include <Arduino.h>

class PIDController
{
public:
  /**
   * @brief Constructor for the PID controller
   *
   * @param[in] kp Proportional gain
   * @param[in] ki Integral gain
   * @param[in] kd Derivative gain
   * @param[in] out_min Minimum output limit (anti-windup)
   * @param[in] out_max Maximum output limit (anti-windup)
   */
  PIDController(
    float kp = 1.2f, float ki = 2.0f, float kd = 0.0f, float out_min = -255.0f,
    float out_max = 255.0f);

  /**
   * @brief Update the PID controller with the current error and timestep
   *
   * @param[in] error The difference between target and measured value
   * @param[in] dt_seconds Time elapsed since last update (seconds)
   * @return The control signal output after applying PID algorithm
   */
  float update(float error, float dt_seconds);

  /**
   * @brief Reset internal state of the PID (integral and previous error)
   */
  void reset();

  /**
   * @brief Set new gain parameters for the PID controller
   */
  void set_gains(float kp, float ki, float kd);

  /**
   * @brief Set output limits to constrain the PID controller output
   */
  void set_output_limits(float min, float max);

private:
  float kp_;            /**< Proportional gain */
  float ki_;            /**< Integral gain */
  float kd_;            /**< Derivative gain */
  float integral_;      /**< Accumulated integral error */
  float prev_error_;    /**< Previous update error for derivative calculation */
  float out_min_;       /**< Minimum controller output limit */
  float out_max_;       /**< Maximum controller output limit */
};


class WheelController
{
public:
  /**
   * @brief Default constructor initializing default PID gains and wheel parameters
   */
  WheelController();

  /**
   * @brief Initialize hardware pins for controlling wheel motor
   * @param[in] forward_pin GPIO for forward motor PWM
   * @param[in] reverse_pin GPIO for reverse motor PWM
   */
  void setup(int forward_pin, int reverse_pin);

  /**
   * @brief Update filtered wheel speed from encoder counts
   * @param[in] delta_counts Change in encoder counts since last update
   * @param[in] dt_seconds Time elapsed since last update (seconds)
   */
  void update_speed(long delta_counts, float dt_seconds);

  /**
   * @brief Set the desired target velocity for the wheel (m/s)
   * @param[in] target_velocity Target speed in meters per second
   */
  void set_target_velocity(float target_velocity);

  /**
   * @brief Apply PID control to reach target velocity based on current speed
   * @param[in] current_velocity Current filtered wheel speed (m/s)
   * @param[in] dt_seconds Time elapsed since last control update (seconds)
   */
  void apply_control(float current_velocity, float dt_seconds);

  /**
   * @brief Get the current filtered wheel speed
   * @return Filtered speed in meters per second
   */
  float get_filtered_speed() const {return filtered_speed_meters_per_sec_;}

  /**
   * @brief Set PID gains
   */
  void set_pid_gains(float kp, float ki, float kd);

  /**
   * @brief Set wheel parameters affecting speed calculations
   * @param[in] wheel_radius Wheel radius in meters
   * @param[in] counts_per_rev Encoder counts per wheel revolution
   */
  void set_wheel_parameters(float wheel_radius, int counts_per_rev);

  /**
   * @brief Configure speed filtering parameters
   * @param[in] ema_alpha Alpha factor for exponential moving average filtering [0..1]
   * @param[in] zero_clamp Speeds below this threshold (m/s) are clamped to zero
   */
  void set_speed_filtering(float ema_alpha, float zero_clamp);

  /**
   * @brief Set minimum and maximum allowable wheel speeds
   * @param[in] min_speed Minimum speed (m/s)
   * @param[in] max_speed Maximum speed (m/s)
   */
  void set_speed_limits(float min_speed, float max_speed);

private:
  /**
   * @brief Apply the PWM signal to motor GPIO pins, handling direction
   * @param[in] pwm Signed PWM value, positive for forward, negative for reverse
   */
  void apply_pwm(int pwm);

  PIDController pid_;              /**< PID controller instance */
  int forward_pin_;                /**< GPIO for forward PWM */
  int reverse_pin_;                /**< GPIO for reverse PWM */
  float wheel_radius_meters_;      /**< Wheel radius in meters */
  int counts_per_revolution_;      /**< Encoder counts per full wheel revolution */
  float ema_alpha_;                /**< EMA filtering strength for speed */
  float zero_clamp_meters_per_sec_; /**< Speed threshold below which value is clamped to zero */
  float filtered_speed_meters_per_sec_; /**< Filtered wheel speed */
  float target_velocity_;          /**< Target wheel velocity */
  float min_speed_;                /**< Minimum allowed wheel speed */
  float max_speed_;                /**< Maximum allowed wheel speed */
};
