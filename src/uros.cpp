#include "uros.h"
#include <cstring>

// Fallback instance pointer
static URos * g_uros_instance = nullptr;

// Service callback for setting parameters
// Note: rclc_service_callback_t doesn't support context, so we use global instance
static void set_param_service_callback(const void * req, void * res)
{
  if (!g_uros_instance || !req || !res) return;

  URos * self = g_uros_instance;

  // Cast request and response
  const rcl_interfaces__srv__SetParameters_Request * request =
    (const rcl_interfaces__srv__SetParameters_Request *)req;
  rcl_interfaces__srv__SetParameters_Response * response =
    (rcl_interfaces__srv__SetParameters_Response *)res;

  // Initialize response safely
  response->results.size = 0;
  response->results.capacity = 0;
  response->results.data = NULL;

  // Validate request
  if (!request->parameters.data || request->parameters.size == 0) {
    return;
  }

  // Allocate response results array
  size_t param_count = request->parameters.size;
  if (param_count > 10) param_count = 10;  // Limit to prevent overflow

  response->results.data = (rcl_interfaces__msg__SetParametersResult *)malloc(
    param_count * sizeof(rcl_interfaces__msg__SetParametersResult));

  if (!response->results.data) {
    return;  // malloc failed
  }

  response->results.size = param_count;
  response->results.capacity = param_count;

  // Process each parameter
  for (size_t i = 0; i < param_count && i < request->parameters.size; i++) {
    const rcl_interfaces__msg__Parameter * current_param = &request->parameters.data[i];
    if (!current_param) continue;

    // Initialize result
    response->results.data[i].successful = false;
    response->results.data[i].reason.data = NULL;
    response->results.data[i].reason.size = 0;
    response->results.data[i].reason.capacity = 0;

    // Check if parameter is a double and has a valid name
    if (current_param->value.type == 3 && current_param->name.data && current_param->name.size > 0) {
      const char * name = current_param->name.data;
      double value = current_param->value.double_value;

      if (strcmp(name, "left_kp") == 0) {
        self->pid_kp_left = value;
        response->results.data[i].successful = true;
      } else if (strcmp(name, "left_ki") == 0) {
        self->pid_ki_left = value;
        response->results.data[i].successful = true;
      } else if (strcmp(name, "left_kd") == 0) {
        self->pid_kd_left = value;
        response->results.data[i].successful = true;
      } else if (strcmp(name, "right_kp") == 0) {
        self->pid_kp_right = value;
        response->results.data[i].successful = true;
      } else if (strcmp(name, "right_ki") == 0) {
        self->pid_ki_right = value;
        response->results.data[i].successful = true;
      } else if (strcmp(name, "right_kd") == 0) {
        self->pid_kd_right = value;
        response->results.data[i].successful = true;
      }
    }
  }
}

URos::URos(){}

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
  // Get the default memory allocator
  allocator = rcl_get_default_allocator();

  // Initialize rclc_support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Initialize node
  RCCHECK(rclc_node_init_default(&node, "microbot_controller_node", "", &support));

  // Create cmd_vel subscriber
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel")
  );

  // Initialize executor for subscription (1 handle)
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));

  // ========== CUSTOM SET_PARAMETERS SERVICE ==========
  g_uros_instance = this;

  // Create set_parameters service
  RCCHECK(rclc_service_init_default(
    &set_param_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(rcl_interfaces, srv, SetParameters),
    "/microbot_controller_node/set_parameters"));

  // Initialize executor for service (1 handle)
  RCCHECK(rclc_executor_init(&executor_params, &support.context, 1, &allocator));

  // Add service to executor
  RCCHECK(rclc_executor_add_service(
    &executor_params,
    &set_param_service,
    &set_param_req,
    &set_param_res,
    set_param_service_callback));
}