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

#pragma once

#include <string>
#include <mutex>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The external torque controller applies torques to the joints received from a
 * ROS2 topic.
 */
class JuliaTorqueController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  JuliaTorqueController();

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  // ROS2 subscriber for torque commands
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_subscriber_;
  std::string torque_topic_name_;

  /**
   * @brief Callback function for the torque subscriber.
   * @param msg The received torque message.
   */
  void on_torque_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  std::string arm_id_;
  std::string robot_description_;
  const int num_joints = 7;

  // Thread-safe storage for the desired torques
  Vector7d desired_torques_;
  std::mutex torque_mutex_;
  rclcpp::Time last_torque_time_;

  // Safety timeout. If no message is received for this duration, apply zero torque.
  const rclcpp::Duration safety_timeout_{std::chrono::milliseconds(100)};
};

}  // namespace franka_example_controllers
