/*
 * Publisher definition and implementation.
 */

#include "../include/speed_control/speed_control.hpp"

#include <rclcpp/node_interfaces/node_parameters.hpp>

/**
 * @brief Creates a Pub node.
 */
//! Call the base class constructor providing a string embedding the node name
//! Initialize other members at will
SpeedControlNode::SpeedControlNode() : Node("speed_control_node")
{
  //! Declare each parameter
  //! Simplest syntax is, for any type:
  //! declare_parameter(NAME_STRING, DEFAULT_VALUE, DESCRIPTOR);
  this->declare_parameter<int>("method", 1);
  this->declare_parameter("tty_device", "");
  this->declare_parameter<int>("extern_reference", 0);

  method = this->get_parameter("method").as_int();
  tty_device = this->get_parameter("tty_device").as_string();
  extern_reference = this->get_parameter("extern_reference").as_int();

  //! Initialize a publisher with create_publisher from the base class:
  //! this->create_publisher<INTERFACE_TYPE>(
  //!   TOPIC_NAME [string],
  //!   PUBLISH_QoS (...),
  //!   ...
  //! );
  //! This object will be used later on to publish messages
  speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/motor1/actual_speed", rclcpp::QoS(10));

  reference_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/motor1/reference_speed_pub", rclcpp::QoS(10));

  //! Initialize a subscriber with create_subscription from the base class:
  //! this->create_subscription<INTERFACE_TYPE>(
  //!   TOPIC_NAME [string],
  //!   SUBSCRIPTION_QoS (...),
  //!   CALLBACK_WRAPPER (with captures and argument placeholders)
  //!   (...)
  //! );
  reference_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "/motor1/reference_speed_sub", rclcpp::QoS(10),
      std::bind(&SpeedControlNode::msg_callback, this, std::placeholders::_1));

  tty_fd = adapter_init(tty_device, baudrate);  // Configurazione porta seriale

  printf("configurazione porta seriale completata\n");

  if (tty_fd == -1)
  {
    exit(EXIT_FAILURE);
  }

  command_settings(tty_fd, speed, CANUSB_MODE_NORMAL, CANUSB_FRAME_STANDARD);
  printf("CANUSB configurata\n");

  usleep(0.1 * 1e6);  // Tempo necessario alla configurazione della pennetta USB

  current_state = request_status_word(tty_fd);

  printf("status richiesto: %d\n", current_state);

  usleep(0.1 * 1e6);

  go_to_operational_enable(tty_fd, current_state);

  usleep(0.1 * 1e6);

  nmt_data = 0x0101;

  frame_send(tty_fd, CANUSB_FRAME_STANDARD, (unsigned char)0x00, (unsigned char)0x00, (unsigned char*)&nmt_data, 2);

  usleep(0.1 * 1e6);

  // MAPPING

  index = 0x60ff;
  subindex = 0x00;
  object_len = 0x20;

  first_entry = index << 16 | subindex << 8 | object_len;

  printf("MAPPING:\n");
  result = mapPDO(tty_fd, node_id, RPDO, 1, (unsigned char*)&first_entry, 1);

  //! Logging macro used to deliver a message to the logging subsystem, INFO level
  RCLCPP_INFO(this->get_logger(), "Publisher initialized");

  supreme_time = this->get_clock()->now();

  if (!extern_reference)
  {
    //! Create and activate a timer with create_wall_timer from the base class
    //! providing an std::chrono::duration as the period and a call wrapper for
    //! the callback, capturing the node object
    //! Since this callback must be void, the wrapper has no arguments specified
    pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(PUB_PERIOD),
                                         std::bind(&SpeedControlNode::pub_timer_callback, this));
  }
}

/**
 * @brief Echoes a new message.
 *
 * @param msg New message.
 */
void SpeedControlNode::msg_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  //! Get the data simply by accessing the object member as the interface specifies
  rpm = msg->data;
  // RCLCPP_INFO(this->get_logger(), msg->data.c_str());
  if (extern_reference)
  {
    publish_speed();
  }
}

void SpeedControlNode::publish_speed()
{
  auto start_time = this->get_clock()->now();

  printf("ref speed: %.3f\n", rpm);

  int actual_internal_rpm = read_velocity_actual_value(tty_fd, 1);

  result = set_speed(tty_fd, rpm, node_id, method);

  if (result == -1)
  {
    exit(EXIT_FAILURE);
  }

  double actual_rpm = internal2rpm(actual_internal_rpm, FEED_CONSTANT, REDUCTION_FACTOR);

  auto end_time = this->get_clock()->now();

  rclcpp::Duration elapsed_time = end_time - start_time;

  printf("speed: %.3f rpm (time: %.3f ms) \n", actual_rpm, elapsed_time.seconds() * 1000.0);

  printf("-----------------------------------------------\n");

  // // Build the new message
  // //! Create a new message of the specific interface type
  // //! It is better to initialize it as empty as below
  std_msgs::msg::Float64 new_speed{};

  // //! Populate the data field of the message using its setter method
  new_speed.set__data(actual_rpm);

  // //! Publish the new message by calling the publish method of the publisher member
  speed_publisher_->publish(new_speed);

  std_msgs::msg::Float64 new_reference{};
  new_reference.set__data(rpm);
  reference_publisher_->publish(new_reference);
}

/**
 * @brief Publishes a message on timer occurrence.
 */
void SpeedControlNode::pub_timer_callback(void)
{
  publish_speed();
}

int main(int argc, char** argv)
{
  //! This automatically creates the global context->DDS participant for this application
  //! and parses all ROS-specific input arguments eventually passed to the new process
  //! (and installs signal handlers)
  rclcpp::init(argc, argv);

  //! Since it's all made of shared pointers, initialize the new node as such
  //! (note: here we're calling an actual constructor but obtaining a smart pointer
  //! to the new object)
  auto speed_control_node = std::make_shared<SpeedControlNode>();

  //! This automatically creates a default, single-threaded executor, adds to it
  //! the jobs defined by the new node, and makes the current thread tend to that
  rclcpp::spin(speed_control_node);

  //! Whatever happens now, does so only after rclcpp::spin has returned

  //! Shut down the global context, the DDS participant and all its
  //! endpoints, effectively terminating the connection to the DDS layer
  //! (note: this doesn't necessarily destroy the corresponding objects)
  rclcpp::shutdown();

  // Just exit
  std::cout << "Node terminated" << std::endl;
  exit(EXIT_SUCCESS);
}
