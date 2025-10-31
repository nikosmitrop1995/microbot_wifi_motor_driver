#include "uros.h"
#include <cstring>

// Parameter change callback used by rclc executor
static bool uros_on_param_changed(const Parameter * old_param, const Parameter * new_param, void * context)
{
  (void)old_param;
  URos * self = static_cast<URos *>(context);
  if (!self || !new_param) {return false;}
  if (new_param->value.type != RCLC_PARAMETER_DOUBLE) {return false;}
  const char * name = new_param->name.data;
  const double v = new_param->value.double_value;
  if (std::strcmp(name, "left.kp") == 0) {
    self->pid_kp_left = v;
  } else if (std::strcmp(name, "left.ki") == 0) {
    self->pid_ki_left = v;
  } else if (std::strcmp(name, "left.kd") == 0) {
    self->pid_kd_left = v;
  } else if (std::strcmp(name, "right.kp") == 0) {
    self->pid_kp_right = v;
  } else if (std::strcmp(name, "right.ki") == 0) {
    self->pid_ki_right = v;
  } else if (std::strcmp(name, "right.kd") == 0) {
    self->pid_kd_right = v;
  }
  return true;
}

URos::URos(){
}

URos::~URos(){}

std::vector<int> URos::split_ip(std::stringstream &ip_address){

  std::string segment;
  std::vector<int> ip_vector;
  while(std::getline(ip_address, segment, '.'))
  {
    ip_vector.push_back(std::stoi(segment));
  }
  return ip_vector;
}

void URos::connect_to_wifi(
  char * ssid, char * psk,
  std::stringstream &ip_address,
  uint16_t &port)
{
  std::vector<int> ip_vector = URos::split_ip(ip_address);
  IPAddress agent_ip(ip_vector[0], ip_vector[1], ip_vector[2], ip_vector[3]);
  set_microros_wifi_transports(ssid, psk, agent_ip, port);
  delay(2000);
}

void URos::initialize(){

  // Get the default memory allocator provided by rcl
  allocator = rcl_get_default_allocator();

  // Initialize rclc_support with default allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Initialize a ROS node with the name "micro_ros_platformio_node"
  RCCHECK(rclc_node_init_default(&node, "microbot_controller_node", "", &support));

  // Create Subscriber
  RCCHECK(rclc_subscription_init_default
  (
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel")
  );

  // Initialize an executor that will manage the execution of all the ROS entities (publishers, subscribers, services, timers)
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));

  // Initialize parameter server (separate executor will drive it)
  RCSOFTCHECK(rclc_parameter_server_init_default(&param_server, &node));

  // Declare PID parameters as doubles (generic add + set initial value)
  RCSOFTCHECK(rclc_add_parameter(&param_server, "left.kp", RCLC_PARAMETER_DOUBLE));
  RCSOFTCHECK(rclc_parameter_set_double(&param_server, "left.kp", pid_kp_left));
  RCSOFTCHECK(rclc_add_parameter(&param_server, "left.ki", RCLC_PARAMETER_DOUBLE));
  RCSOFTCHECK(rclc_parameter_set_double(&param_server, "left.ki", pid_ki_left));
  RCSOFTCHECK(rclc_add_parameter(&param_server, "left.kd", RCLC_PARAMETER_DOUBLE));
  RCSOFTCHECK(rclc_parameter_set_double(&param_server, "left.kd", pid_kd_left));
  RCSOFTCHECK(rclc_add_parameter(&param_server, "right.kp", RCLC_PARAMETER_DOUBLE));
  RCSOFTCHECK(rclc_parameter_set_double(&param_server, "right.kp", pid_kp_right));
  RCSOFTCHECK(rclc_add_parameter(&param_server, "right.ki", RCLC_PARAMETER_DOUBLE));
  RCSOFTCHECK(rclc_parameter_set_double(&param_server, "right.ki", pid_ki_right));
  RCSOFTCHECK(rclc_add_parameter(&param_server, "right.kd", RCLC_PARAMETER_DOUBLE));
  RCSOFTCHECK(rclc_parameter_set_double(&param_server, "right.kd", pid_kd_right));

  // Dedicated executor for parameter server
  RCCHECK(rclc_executor_init(&executor_params, &support.context, 1, &allocator));
  RCSOFTCHECK(rclc_executor_add_parameter_server(&executor_params, &param_server, uros_on_param_changed, this));
}
