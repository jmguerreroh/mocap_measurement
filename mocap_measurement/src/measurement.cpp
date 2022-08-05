/*
# Copyright (c) 2022 José Miguel Guerrero Hernández
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "mocap_msgs/msg/rigid_body.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mocap_measurement_msgs/msg/measurement.hpp"

class MeasurementSubscriber : public rclcpp::Node
{
  public:
    MeasurementSubscriber() : Node("measurement_subscriber")
    {
      auto qos = rclcpp::QoS( rclcpp::QoSInitialization( RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5 ));
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
   
      subscription_rb_ = this->create_subscription<mocap_msgs::msg::RigidBody>(
      "/rigid_bodies", qos, std::bind(&MeasurementSubscriber::topic_callback_rb, this, std::placeholders::_1));

      subscription_amcl_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", qos, std::bind(&MeasurementSubscriber::topic_callback_amcl, this, std::placeholders::_1));

      publisher_measurement_ = this->create_publisher<mocap_measurement_msgs::msg::Measurement>(
      "/measurement", 1000);

      publisher_rb_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/rb_pose", 1000);

    }


  private:
    void topic_callback_rb(const mocap_msgs::msg::RigidBody msg) const
    {     
      //std::cout << "X: " << msg.pose.position.x << ", Y: " << msg.pose.position.y << ", Z: " << msg.pose.position.z << std::endl;
      //std::cout << "rX: " << msg.pose.orientation.x << ", rY: " << msg.pose.orientation.y << ", rZ: " << msg.pose.orientation.z << ", rW: " << msg.pose.orientation.w << std::endl;
      geometry_msgs::msg::PoseWithCovarianceStamped pos;
      pos.header = msg.header;
      pos.header.frame_id = "map";
      pos.pose.pose = msg.pose;
      for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
          pos.pose.covariance[i * 6 + j] = 0.0;
        }
      }

      MeasurementSubscriber::rigidbody_ = pos;
      publisher_rb_ -> publish(pos);
      
      update();
    }

    void topic_callback_amcl(const geometry_msgs::msg::PoseWithCovarianceStamped msg) const
    {   
      //std::cout << "X: " << msg.markers[0].pose.position.x << ", Y: " << msg.markers[0].pose.position.y << ", Z: " << msg.markers[0].pose.position.z << std::endl;
      //std::cout << "rX: " << msg.markers[0].pose.orientation.x << ", rY: " << msg.markers[0].pose.orientation.y << ", rZ: " << msg.markers[0].pose.orientation.z << ", rW: " << msg.markers[0].pose.orientation.w << std::endl;  

      MeasurementSubscriber::amcl_ = msg;

      update();
    }


    void update() const
    {
      mocap_measurement_msgs::msg::Measurement measurement;
      measurement.header.stamp = now();
      measurement.header.frame_id = "map";
      measurement.rb_pose = MeasurementSubscriber::rigidbody_;
      measurement.amcl_pose = MeasurementSubscriber::amcl_;

      publisher_measurement_ -> publish(measurement);
    }

    rclcpp::Subscription<mocap_msgs::msg::RigidBody>::SharedPtr subscription_rb_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_amcl_;
    rclcpp::Publisher<mocap_measurement_msgs::msg::Measurement>::SharedPtr publisher_measurement_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_rb_;

    static geometry_msgs::msg::PoseWithCovarianceStamped rigidbody_;
    static geometry_msgs::msg::PoseWithCovarianceStamped amcl_;
};

geometry_msgs::msg::PoseWithCovarianceStamped MeasurementSubscriber::rigidbody_;
geometry_msgs::msg::PoseWithCovarianceStamped MeasurementSubscriber::amcl_;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MeasurementSubscriber>());
  rclcpp::shutdown();
  return 0;
}