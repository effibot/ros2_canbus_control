/*
 * Publisher definition.
 */

#ifndef SPEED_CONTROL_HPP
#define SPEED_CONTROL_HPP

#include <iostream>
#include <thread>
#include <vector>
#include <signal.h>
#include <termios.h>

#include "../include/readStatusWord.hpp"
#include "../include/frameManager.hpp"
#include "../include/tools.hpp"

#include <rclcpp/rclcpp.hpp> //! rclcpp base library

#include "std_msgs/msg/float64.hpp"//! Interface library that we'll use
#include "std_msgs/msg/int32.hpp"//! Interface library that we'll use

//! We'll see how to properly manage this kind of "parameters"
#define PUB_PERIOD 25 // Publisher transmission time period [ms]

/**
 * Simple publisher node: transmits strings on a topic.
 */
//! Every node must extend publicly the Node base class
class SpeedControlNode : public rclcpp::Node
{
public:
  //! There must always be a constructor, with arbitrary input arguments
  SpeedControlNode();

  //! ROS-specific members better be private
private:
  //! DDS endpoint, acting as a publisher
  //! Syntax is: rclcpp::Publisher<INTERFACE_TYPE>::SharedPtr OBJ;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr reference_publisher_;

  //! DDS endpoint, acting as a subscriber
  //! When a message is received, a callback job is issued, which better be a private method
  //! Syntax is: rclcpp::Subscription<INTERFACE_TYPE>::SharedPtr OBJ;
  //! Callback signature must be:
  //!   void FUNC_NAME(const INTERFACE_TYPE::SharedPtr ARG_NAME);
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr reference_subscriber_;
  void msg_callback(const std_msgs::msg::Float64::SharedPtr msg);

  //! ROS-2 managed timer: enables one to set up a periodic job
  //! The job is coded in a callback, which better be a private method
  //! Syntax is: rclcpp::TimerBase::SharedPtr OBJ;
  //! Callback signature must be: void FUNC_NAME(void);
  rclcpp::TimerBase::SharedPtr pub_timer_;
  void pub_timer_callback(void);

  rcl_interfaces::msg::ParameterDescriptor param_descriptor_; //! Number parameter descriptor

  int result;
  std::string tty_device;
  int tty_fd;
  int baudrate = CANUSB_TTY_BAUD_RATE_DEFAULT;
  CANUSB_SPEED speed = CANUSB_SPEED_500000; // Il Baud Rate è configurato con questo valore

  int nmt_data;

  int method;

  int node_id = 1;

  int extern_reference;

  double rpm = 5;
  int rpm_size = 10;

  short index;
  unsigned char subindex;
  unsigned char object_len;

  DSP402_STATES current_state;

  int first_entry;

  rclcpp::Time supreme_time;

  void publish_speed();

};

#endif
