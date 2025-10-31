#include "driver.h"
#include "uros.h"


// Declare uros pointer
URos * uros = nullptr;

Driver driver;

unsigned long last_update_time = 0;
const unsigned long update_interval_ms = 20;  // 50 Hz control loop

void initialize_wifi_connection(const char* ssid, const char* psk)
{
  // Initialize LED pin
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Initialize WiFi in station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);  // Disconnect from any previous WiFi
  delay(100);

  // Start WiFi connection
  WiFi.begin(ssid, psk);

  // Wait for connection with dots progress
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {  // 20 * 500ms = 10 second timeout
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Blink LED while connecting
    delay(500);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);
    WiFi.disconnect(true);
    delay(100);
  }
}

void setup()
{
  uros = new URos;
  char ssid[] = "";
  char psk[] = "";
  initialize_wifi_connection(ssid, psk);

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

  driver.setup();
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
