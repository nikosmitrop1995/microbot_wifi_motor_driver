#include <Arduino.h>
#include <WiFi.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl_interfaces/srv/set_parameters.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <vector>


#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      while(1) {                 \
        delay(100);              \
      }                          \
    }                            \
  }

#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

/**
 * @brief uROS class has everything related to micro-ROS
 */

class URos
{
public:
  URos();

  ~URos();

  static std::vector<int> split_ip(std::stringstream &ip_address);

  static void connect_to_wifi(char * ssid,char * psk, std::stringstream &ip_address, uint16_t &port);

  void initialize();

  rcl_node_t node;
  rclc_support_t support;
  rcl_allocator_t allocator;

  // Message
  geometry_msgs__msg__Twist msg;

  // Subscriber
  rcl_subscription_t cmd_vel_subscriber;

  // Executor
  rclc_executor_t executor_sub;

  // PID parameters (separate per wheel)
  double pid_kp_left;
  double pid_ki_left;
  double pid_kd_left;
  double pid_kp_right;
  double pid_ki_right;
  double pid_kd_right;

  // Custom set_parameters service
  rcl_service_t set_param_service;
  rcl_interfaces__srv__SetParameters_Request set_param_req;
  rcl_interfaces__srv__SetParameters_Response set_param_res;
  rclc_executor_t executor_params;
};