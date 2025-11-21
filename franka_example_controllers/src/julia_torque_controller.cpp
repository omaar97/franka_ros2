// Copyright (c) 2023 Franka Robotics GmbH
// (Modified by Gemini to create an external torque controller)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "franka_example_controllers/julia_torque_controller.hpp"
#include <franka_example_controllers/robot_utils.hpp> // Re-using robot_utils

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

JuliaTorqueController::JuliaTorqueController()
    : controller_interface::ControllerInterface() {
  desired_torques_.setZero();
}

controller_interface::InterfaceConfiguration
JuliaTorqueController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Claim effort command interfaces for all 7 joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JuliaTorqueController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Claim state interfaces for all 7 joints (position and velocity)
  // Even if not used in this specific controller, it's good practice
  // and often required by the hardware interface.
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type JuliaTorqueController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/) {

  Vector7d torques_to_apply;
  torques_to_apply.setZero(); // Default to zero torques

  {
    std::lock_guard<std::mutex> lock(torque_mutex_);
    
    // Check if the last received message is recent enough
    if ((time - last_torque_time_) < safety_timeout_) {
      torques_to_apply = desired_torques_;
    } 
    // If not recent, torques_to_apply remains zero (safety)
  }

  // Apply the torques (either desired or zero) to the hardware
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(torques_to_apply(i));
  }
  return controller_interface::return_type::OK;
}

void JuliaTorqueController::on_torque_command(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  // Check if the message has the correct number of joints
  if (msg->data.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Received torque command with %zu elements, expected %d.",
                 msg->data.size(), num_joints);
    return;
  }

  // Write the new torques to the member variable under lock
  std::lock_guard<std::mutex> lock(torque_mutex_);
  for (int i = 0; i < num_joints; ++i) {
    desired_torques_(i) = msg->data[i];
  }
  // Update the timestamp of the last received message
  last_torque_time_ = this->get_node()->get_clock()->now();
}

CallbackReturn JuliaTorqueController::on_init() {
  try {
    // Declare parameters
    auto_declare<std::string>("arm_id", "");
    auto_declare<std::string>("torque_topic", "~/external_torques");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JuliaTorqueController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // Get arm_id
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  
  // Get robot description to extract arm_id if not provided
  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();
  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }
  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  // Get torque topic name
  torque_topic_name_ = get_node()->get_parameter("torque_topic").as_string();

  // Initialize the subscriber
  torque_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      torque_topic_name_, 10,
      std::bind(&JuliaTorqueController::on_torque_command, this,
                std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "Configured JuliaTorqueController for arm_id '%s' on topic '%s'",
              arm_id_.c_str(), torque_topic_name_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn JuliaTorqueController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Reset desired torques to zero on activation
  std::lock_guard<std::mutex> lock(torque_mutex_);
  desired_torques_.setZero();
  // Set last_torque_time_ to a very old value (or 0) to ensure
  // zero torques are applied until a new message is received.
  last_torque_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());

  return CallbackReturn::SUCCESS;
}

CallbackReturn JuliaTorqueController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Reset the subscriber to stop receiving messages
  torque_subscriber_.reset();
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JuliaTorqueController,
                       controller_interface::ControllerInterface)