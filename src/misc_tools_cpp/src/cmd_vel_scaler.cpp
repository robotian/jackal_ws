/*
 * Copyright (C) 2025 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <memory>
#include <chrono>

using std::placeholders::_1;

/**
 * @class CmdVelScaler
 * @brief Subscribes to a Twist topic, scales the velocities, and republishes.
 *
 * This node subscribes to the 'cmd_vel_nav' topic, multiplies the
 * linear and angular velocities by configurable scaling factors,
 * and publishes the result to the 'cmd_vel' topic.
 *
 * Parameters:
 * - "scale_linear" (double): The factor to scale linear velocities (x, y, z).
 * (Default: 1.0)
 * - "scale_angular" (double): The factor to scale angular velocities (x, y, z).
 * (Default: 1.0)
 *
 * Subscribes:
 * - "cmd_vel_nav" (geometry_msgs/msg/Twist): Input velocity commands.
 *
 * Publishes:
 * - "cmd_vel" (geometry_msgs/msg/Twist): Scaled output velocity commands.
 */
class CmdVelScaler : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the CmdVelScaler node.
   *
   * Initializes the node, declares and retrieves scaling parameters,
   * sets up the publisher, and creates the subscription.
   */
  CmdVelScaler()
  : Node("cmd_vel_scaler_node")
  {
    // Declare parameters with default values
    this->declare_parameter<double>("scale_linear", 1.0);
    this->declare_parameter<double>("scale_angular", 1.0);
    this->declare_parameter<std::string>("output_cmd_vel_topic", "cmd_vel");
    this->declare_parameter<std::string>("input_cmd_vel_topic", "cmd_vel_nav");
    this->declare_parameter<double>("deadband_linear", 0.0);
    this->declare_parameter<double>("deadband_angular", 0.0);
    this->declare_parameter<double>("vector_magnitude_threshold", 10.0);

    // Read the parameters into member variables
    this->get_parameter("scale_linear", scale_linear_);
    this->get_parameter("scale_angular", scale_angular_);
    this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
    this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
    this->get_parameter("deadband_linear", deadband_linear_);
    this->get_parameter("deadband_angular", deadband_angular_);
    this->get_parameter("vector_magnitude_threshold", vector_mag_threshold_);

    RCLCPP_INFO(this->get_logger(), "Starting velocity scaler node with linear scale: %.2f, angular scale: %.2f",
      scale_linear_, scale_angular_);

    // Create the publisher for the '/cmd_vel' topic
    // publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 30);
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 30);

    // Create the subscription to the '/cmd_vel_nav' topic
    // subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    //   "cmd_vel_nav", 1, std::bind(&CmdVelScaler::topic_callback, this, _1));
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav", 1, std::bind(&CmdVelScaler::topic_callback, this, _1));
  }

private:
  /**
   * @brief Callback function for the 'cmd_vel_nav' subscription.
   *
   * This function is called whenever a message is received on the
   * 'cmd_vel_nav' topic. It scales the message and republishes it.
   *
   * @param msg The incoming Twist message.
   */
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Create a new Twist message for the scaled output
    auto scaled_msg = geometry_msgs::msg::Twist();

    double small_value = 0.0001;
    // double vector_mag_threshold = 0.2;
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;
    double scaled_linear_x = linear_x* scale_linear_;
    double scaled_angular_z = angular_z * scale_angular_;

    if(std::abs(scaled_angular_z) < small_value && std::abs(scaled_linear_x) > small_value) {  // consider it as translational motion
      scaled_angular_z = 0.0;
      if (std::abs(scaled_linear_x) < deadband_linear_ ) {
        scaled_linear_x = deadband_linear_;
        scaled_angular_z = 0.0;
      }
    }else if(std::abs(scaled_linear_x) < small_value && std::abs(scaled_angular_z) > small_value) {  // consider it as rotational motion
      scaled_linear_x = 0.0;
      if (std::abs(scaled_angular_z) < deadband_angular_ ) {
        scaled_angular_z = deadband_angular_;
        scaled_linear_x = 0.0;
      }
    }else if(std::abs(scaled_linear_x) < small_value && std::abs(scaled_angular_z) < small_value) {  // consider it as no motion
      scaled_linear_x = 0.0;
      scaled_angular_z = 0.0;
    }else{
      
      double vector_magnitude =std::sqrt(scaled_angular_z*scaled_angular_z + scaled_linear_x*scaled_linear_x);
      if(vector_magnitude < vector_mag_threshold_) {
        double radius = std::abs(scaled_linear_x / scaled_angular_z);
        double new_omega = vector_mag_threshold_ / std::sqrt(1 + radius*radius);
        double new_v = new_omega * radius;
        scaled_linear_x = (scaled_linear_x > 0) ? new_v : -new_v;
        scaled_angular_z = (scaled_angular_z > 0) ? new_omega : -new_omega;
      }      
    }

    // Apply the scaling factors
    scaled_msg.linear.x = scaled_linear_x;
    scaled_msg.linear.y = 0.0;
    scaled_msg.linear.z = 0.0;

    scaled_msg.angular.x = 0.0;
    scaled_msg.angular.y = 0.0;
    scaled_msg.angular.z = scaled_angular_z;

    // Publish the scaled message
    publisher_->publish(scaled_msg);
  }



  /**
   * @brief Callback function for the 'cmd_vel_nav' subscription.
   *
   * This function is called whenever a message is received on the
   * 'cmd_vel_nav' topic. It scales the message and republishes it.
   *
   * @param msg The incoming Twist message.
   */
  // void topic_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  // {
  //   // Create a new Twist message for the scaled output
  //   auto scaled_msg = geometry_msgs::msg::Twist();

  //   double small_value = 0.0001;
  //   // double vector_mag_threshold = 0.2;
  //   double linear_x = msg->twist.linear.x;
  //   double angular_z = msg->twist.angular.z;
  //   double scaled_linear_x = linear_x* scale_linear_;
  //   double scaled_angular_z = angular_z * scale_angular_;

  //   if(std::abs(scaled_angular_z) < small_value && std::abs(scaled_linear_x) > small_value) {  // consider it as translational motion
  //     scaled_angular_z = 0.0;
  //     if (std::abs(scaled_linear_x) < deadband_linear_ ) {
  //       scaled_linear_x = deadband_linear_;
  //       scaled_angular_z = 0.0;
  //     }
  //   }else if(std::abs(scaled_linear_x) < small_value && std::abs(scaled_angular_z) > small_value) {  // consider it as rotational motion
  //     scaled_linear_x = 0.0;
  //     if (std::abs(scaled_angular_z) < deadband_angular_ ) {
  //       scaled_angular_z = deadband_angular_;
  //       scaled_linear_x = 0.0;
  //     }
  //   }else if(std::abs(scaled_linear_x) < small_value && std::abs(scaled_angular_z) < small_value) {  // consider it as no motion
  //     scaled_linear_x = 0.0;
  //     scaled_angular_z = 0.0;
  //   }else{
      
  //     double vector_magnitude =std::sqrt(scaled_angular_z*scaled_angular_z + scaled_linear_x*scaled_linear_x);
  //     if(vector_magnitude < vector_mag_threshold_) {
  //       double radius = std::abs(scaled_linear_x / scaled_angular_z);
  //       double new_omega = vector_mag_threshold_ / std::sqrt(1 + radius*radius);
  //       double new_v = new_omega * radius;
  //       scaled_linear_x = (scaled_linear_x > 0) ? new_v : -new_v;
  //       scaled_angular_z = (scaled_angular_z > 0) ? new_omega : -new_omega;
  //     }      
  //   }

  //   // Apply the scaling factors
  //   scaled_msg.linear.x = scaled_linear_x;
  //   scaled_msg.linear.y = 0.0;
  //   scaled_msg.linear.z = 0.0;

  //   scaled_msg.angular.x = 0.0;
  //   scaled_msg.angular.y = 0.0;
  //   scaled_msg.angular.z = scaled_angular_z;

  //   // Publish the scaled message
  //   publisher_->publish(scaled_msg);
  // }

  // Member variables
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  // rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
  // rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  double scale_linear_;
  double scale_angular_;
  std::string output_cmd_vel_topic_;
  std::string input_cmd_vel_topic_;
  double deadband_linear_;
  double deadband_angular_;
  double vector_mag_threshold_;
};

/**
 * @brief Main function to initialize and run the node.
 *
 * @param argc Number of command-line arguments.
 * @param argv Command-line arguments.
 * @return int Exit code.
 */
int main(int argc, char * argv[])
{
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create and spin the node
  rclcpp::spin(std::make_shared<CmdVelScaler>());

  // Shut down ROS 2
  rclcpp::shutdown();
  return 0;
}
