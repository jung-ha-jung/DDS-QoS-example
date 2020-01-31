/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Pyo */

#include "publisher/counter.hpp"

using namespace robotis;

// https://en.cppreference.com/w/cpp/symbol_index/chrono_literals
using namespace std::chrono_literals;

Counter::Counter(const std::string & comment, const int32_t & qos_profile)
: Node("counter")
{
  RCLCPP_DEBUG(this->get_logger(), "Test debug message");

  // Declare parameters that may be set on this node
  // https://index.ros.org/doc/ros2/Releases/Release-Dashing-Diademata/#declaring-a-parameter-with-a-parameterdescriptor
  this->declare_parameter(
    "comment",
    rclcpp::ParameterValue("No comments"),
    rcl_interfaces::msg::ParameterDescriptor());

  // Get parameter from yaml
  this->get_parameter("comment");

  auto qos = get_qos(qos_profile);
  pub_ = this->create_publisher<examples_msgs::msg::Count>("count", qos);

  // Rambda function (https://en.cppreference.com/w/cpp/language/lambda)
  timer_ = this->create_wall_timer(
    1s,
    [this, comment]() -> void
      {
        msg_ = std::make_unique<examples_msgs::msg::Count>();
        msg_->header.stamp = this->now();
        msg_->count = count_++;
        RCLCPP_INFO(this->get_logger(), "[%s] Counting: '%d'", comment.c_str(), msg_->count);

        // 'move' let tranfer unique_ptr's ownership
        pub_->publish(std::move(msg_));
      }
    );
}

rclcpp::QoS Counter::get_qos(const int32_t & qos_profile)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  if (qos_profile == 0)
  {
    qos = qos;
  }
  else if (qos_profile == 1)
  {
    qos = rclcpp::QoS(rclcpp::SensorDataQoS());
  }
  else if (qos_profile == 2)
  {
    qos = rclcpp::QoS(rclcpp::ParametersQoS());
  }
  else if (qos_profile == 3)
  {
    qos = rclcpp::QoS(rclcpp::ServicesQoS());
  }
  else if (qos_profile == 4)
  {
    qos = rclcpp::QoS(rclcpp::ParameterEventsQoS());
  }
  else if (qos_profile == 5)
  {
    qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  }
  else if (qos_profile == 6) // Reliablity - reliable
  {
    qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    RCLCPP_INFO(this->get_logger(), "Publisher - reliable...");
  }
  else if (qos_profile == 7) // Reliablity - best_effort
  {
    qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    RCLCPP_INFO(this->get_logger(), "Publisher - best_effort...");
  }
  else if (qos_profile == 8) // Durability - transient_local
  {
    qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    RCLCPP_INFO(this->get_logger(), "Publisher - transient_local...");
  }
  else if (qos_profile == 9) // Durability - volatile
  {
    qos = rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile();
    RCLCPP_INFO(this->get_logger(), "Publisher - volatile...");
  }
  else if (qos_profile == 10) // Durability - KeepAll - transient_local
  {
    qos = rclcpp::QoS(rclcpp::KeepAll()).transient_local();
    RCLCPP_INFO(this->get_logger(), "Publisher - KeepAll - transient_local...");
  }
  else if (qos_profile == 11) // Deadline - 1000ms
  {
    qos = rclcpp::QoS(rclcpp::KeepLast(10)).deadline(1000ms);
    RCLCPP_INFO(this->get_logger(), "Publisher - Deadline - 1000ms...");
  }
  else if (qos_profile == 12) // Deadline - 2000ms
  {
    qos = rclcpp::QoS(rclcpp::KeepLast(10)).deadline(2000ms);
    RCLCPP_INFO(this->get_logger(), "Publisher - Deadline - 2000ms...");
  }
  else
  {
    qos = qos;
  }

  return qos;
}
