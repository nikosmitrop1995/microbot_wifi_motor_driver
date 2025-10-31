#include "pid.h"
#include <geometry_msgs/msg/twist.h>

// Define the control inputs
#define MOT_AIN1_FWD A1
#define MOT_AIN2_REV A2
#define MOT_BIN1_FWD A3
#define MOT_BIN2_REV A4
#define SLP D5

// Define encoder pins
#define ENC_LEFT_A D9
#define ENC_LEFT_B D10
#define ENC_RIGHT_A D7
#define ENC_RIGHT_B D8


enum WheelPosition {
  LEFT_WHEEL,
  RIGHT_WHEEL
};

class Driver
{
public:
  /**
   * @brief Default constructor
   */
  Driver();

  /**
   * @brief Default destructor
   */
  virtual ~Driver() = default;

  /**
   * @brief Subscription callback executed when receiving a message
   *
   * @param[in] msgin Callback message
   */
  static void cmd_vel_callback(const void * msgin);

  /**
   * @brief Setup function
   *
   * Initializes motor drivers, wheel controllers, encoder pins, and interrupts,
   * preparing the driver for motion control and feedback processing.
   */
  void setup();

  /**
   * Set the PID gains for the specified wheel.
   *
   * @param wheel_position The wheel to configure (LEFT_WHEEL or RIGHT_WHEEL).
   * @param kp Proportional gain.
   * @param ki Integral gain.
   * @param kd Derivative gain.
   *
   * Allows independent tuning of each wheel's PID controller.
   */
  void set_pid_gains(WheelPosition wheel_position, float kp, float ki, float kd);

  /**
   * @brief Periodic control update
   *
   * Runs the 50 Hz control loop by calculating wheel speeds from encoder counts,
   * applying PID control to maintain target velocities, and updating tracking
   * variables for the next cycle.
   */
  void update();

private:
  static WheelController left_wheel_;
  static WheelController right_wheel_;

  // Encoder tracking
  static volatile long left_encoder_count_;
  static volatile long right_encoder_count_;
  static volatile long last_left_count_;
  static volatile long last_right_count_;
  static unsigned long last_update_time_;

  static constexpr float wheel_radius_meters = 0.0598f;
  static constexpr float wheel_base_meters = 0.109f;
  static constexpr int counts_per_rev = 1954;

  /**
   * @brief Interrupt Service Routine (ISR) for the left wheel encoder.
   *
   * Reads encoder signals to detect rotation direction and updates the
   * left wheel's encoder count accordingly.
   */
  static void left_encoder_isr();

  /**
   * @brief Interrupt Service Routine (ISR) for the right wheel encoder.
   *
   * Reads encoder signals to detect rotation direction and updates the
   * right wheel's encoder count accordingly.
   */
  static void right_encoder_isr();
};
