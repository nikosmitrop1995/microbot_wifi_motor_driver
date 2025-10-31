#include "driver.h"
#include "uros.h"


// Declare uros pointer
URos * uros = nullptr;

Driver driver;

unsigned long last_update_time = 0;
const unsigned long update_interval_ms = 20;  // 50 Hz control loop

void setup()
{
  driver.setup();

  // Allocate memory for uros object
  uros = new URos;
  char ssid[] = "";
  char psk[] = "";
  std::stringstream ip_address("192.168.1.123");
  uint16_t port = 8888;
  uros->connect_to_wifi(ssid, psk, ip_address, port);

  // Set initial PID defaults BEFORE initialize()
  uros->pid_kp_left = 10.0;
  uros->pid_ki_left = 0.0;
  uros->pid_kd_left = 0.0;
  uros->pid_kp_right = 10.0;
  uros->pid_ki_right = 0.0;
  uros->pid_kd_right = 0.0;

  uros->initialize();
  // Add subscription executor (execute callback when new data received)
  RCCHECK(
    rclc_executor_add_subscription(
      &(uros->executor_sub), &(uros->cmd_vel_subscriber),
      &(uros->msg), &driver.cmd_vel_callback, ON_NEW_DATA));
}

void loop()
{
  // Check if uros is initialized
  if (uros) {
    rclc_executor_spin_some(&(uros->executor_sub), RCL_MS_TO_NS(1));
    rclc_executor_spin_some(&(uros->executor_params), RCL_MS_TO_NS(1));
  }

  unsigned long now = millis();
  if (now - last_update_time >= update_interval_ms) {
    last_update_time = now;
    // Sync PID gains from parameters to the driver before control update
    driver.set_pid_gains(
      LEFT_WHEEL,
      static_cast<float>(uros->pid_kp_left),
      static_cast<float>(uros->pid_ki_left),
      static_cast<float>(uros->pid_kd_left));
    driver.set_pid_gains(
      RIGHT_WHEEL,
      static_cast<float>(uros->pid_kp_right),
      static_cast<float>(uros->pid_ki_right),
      static_cast<float>(uros->pid_kd_right));
    driver.update();
  }
}
