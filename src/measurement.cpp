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
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class MeasurementSubscriber : public rclcpp::Node
{
  public:
    MeasurementSubscriber() : Node("measurement_subscriber")
    {
      auto qos = rclcpp::QoS( rclcpp::QoSInitialization( RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5 ));
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
   

      subscription_rb_ = this->create_subscription<mocap_msgs::msg::RigidBody>(
      "/rigid_bodies", qos, std::bind(&MeasurementSubscriber::topic_callback_rb, this, std::placeholders::_1));

      subscription_amcl_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/poses", qos, std::bind(&MeasurementSubscriber::topic_callback_amcl, this, std::placeholders::_1));

      publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
      "/measurement", qos);
    }


  private:
    void topic_callback_rb(const mocap_msgs::msg::RigidBody msg) const
    {     
        //std::cout << "X: " << msg.pose.position.x << ", Y: " << msg.pose.position.y << ", Z: " << msg.pose.position.z << std::endl;
        //std::cout << "rX: " << msg.pose.orientation.x << ", rY: " << msg.pose.orientation.y << ", rZ: " << msg.pose.orientation.z << ", rW: " << msg.pose.orientation.w << std::endl;
        MeasurementSubscriber::rigidbody_ = msg.pose;
        update();
    }

    void topic_callback_amcl(const visualization_msgs::msg::MarkerArray msg) const
    {   

        //std::cout << "X: " << msg.markers[0].pose.position.x << ", Y: " << msg.markers[0].pose.position.y << ", Z: " << msg.markers[0].pose.position.z << std::endl;
        //std::cout << "rX: " << msg.markers[0].pose.orientation.x << ", rY: " << msg.markers[0].pose.orientation.y << ", rZ: " << msg.markers[0].pose.orientation.z << ", rW: " << msg.markers[0].pose.orientation.w << std::endl;  
        MeasurementSubscriber::amcl_ = msg.markers[0].pose;
        update();
    }

    void update() const
    {
        geometry_msgs::msg::Pose rb = MeasurementSubscriber::rigidbody_;
        geometry_msgs::msg::Pose amcl = MeasurementSubscriber::amcl_;
        geometry_msgs::msg::Pose diff;
        diff.position.x = rb.position.x - amcl.position.x;
        diff.position.y = rb.position.y - amcl.position.y;
        diff.position.z = rb.position.z - amcl.position.z;
        diff.orientation.x = rb.orientation.x - amcl.orientation.x;
        diff.orientation.y = rb.orientation.y - amcl.orientation.y;
        diff.orientation.z = rb.orientation.z - amcl.orientation.z;
        diff.orientation.w = rb.orientation.w - amcl.orientation.w;

        publisher_ -> publish(diff);
    }


    rclcpp::Subscription<mocap_msgs::msg::RigidBody>::SharedPtr subscription_rb_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_amcl_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;

    static geometry_msgs::msg::Pose rigidbody_;
    static geometry_msgs::msg::Pose amcl_;
};

geometry_msgs::msg::Pose MeasurementSubscriber::rigidbody_;
geometry_msgs::msg::Pose MeasurementSubscriber::amcl_;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MeasurementSubscriber>());
  rclcpp::shutdown();
  return 0;
}