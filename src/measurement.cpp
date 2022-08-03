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
#include "geometry_msgs/msg/pose_stamped.hpp"
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

      publisher_diff_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/amcl_rb_diff", 1000);

      publisher_amcl_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/amcl_mean", 1000);

      publisher_rb_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/amcl_rb", 1000);
    }


  private:
    void topic_callback_rb(const mocap_msgs::msg::RigidBody msg) const
    {     
      //std::cout << "X: " << msg.pose.position.x << ", Y: " << msg.pose.position.y << ", Z: " << msg.pose.position.z << std::endl;
      //std::cout << "rX: " << msg.pose.orientation.x << ", rY: " << msg.pose.orientation.y << ", rZ: " << msg.pose.orientation.z << ", rW: " << msg.pose.orientation.w << std::endl;
      geometry_msgs::msg::PoseStamped pos;
      pos.header = msg.header;
      pos.header.frame_id = "map";
      pos.pose = msg.pose;
      MeasurementSubscriber::rigidbody_ = pos;
      publisher_rb_ -> publish(pos);

      update();
    }

    void topic_callback_amcl(const visualization_msgs::msg::MarkerArray msg) const
    {   
      //std::cout << "X: " << msg.markers[0].pose.position.x << ", Y: " << msg.markers[0].pose.position.y << ", Z: " << msg.markers[0].pose.position.z << std::endl;
      //std::cout << "rX: " << msg.markers[0].pose.orientation.x << ", rY: " << msg.markers[0].pose.orientation.y << ", rZ: " << msg.markers[0].pose.orientation.z << ", rW: " << msg.markers[0].pose.orientation.w << std::endl;  
      geometry_msgs::msg::PoseStamped mean;
      mean.header = msg.markers[0].header;

      int n_amcl = 0;
      for (visualization_msgs::msg::Marker amcl_pt : msg.markers) {
        mean.pose.position.x += amcl_pt.pose.position.x;
        mean.pose.position.y += amcl_pt.pose.position.y;
        mean.pose.position.z += amcl_pt.pose.position.z;
        mean.pose.orientation.x += amcl_pt.pose.orientation.x;
        mean.pose.orientation.y += amcl_pt.pose.orientation.y;
        mean.pose.orientation.z += amcl_pt.pose.orientation.z;
        mean.pose.orientation.w += amcl_pt.pose.orientation.w;
        n_amcl++;
      }

      mean.pose.position.x /= n_amcl;
      mean.pose.position.y /= n_amcl;
      mean.pose.position.z /= n_amcl;
      mean.pose.orientation.x /= n_amcl;
      mean.pose.orientation.y /= n_amcl;
      mean.pose.orientation.z /= n_amcl;
      mean.pose.orientation.w /= n_amcl;

      MeasurementSubscriber::amcl_ = mean;
      publisher_amcl_ -> publish(mean);

      update();
    }

    void update() const
    {
      geometry_msgs::msg::PoseStamped rb = MeasurementSubscriber::rigidbody_;
      geometry_msgs::msg::PoseStamped amcl = MeasurementSubscriber::amcl_;
      geometry_msgs::msg::PoseStamped diff;
      diff.header = amcl.header;
      diff.pose.position.x = rb.pose.position.x - amcl.pose.position.x;
      diff.pose.position.y = rb.pose.position.y - amcl.pose.position.y;
      diff.pose.position.z = rb.pose.position.z - amcl.pose.position.z;
      diff.pose.orientation.x = rb.pose.orientation.x - amcl.pose.orientation.x;
      diff.pose.orientation.y = rb.pose.orientation.y - amcl.pose.orientation.y;
      diff.pose.orientation.z = rb.pose.orientation.z - amcl.pose.orientation.z;
      diff.pose.orientation.w = rb.pose.orientation.w - amcl.pose.orientation.w;

      publisher_diff_ -> publish(diff);
    }

    rclcpp::Subscription<mocap_msgs::msg::RigidBody>::SharedPtr subscription_rb_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_amcl_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_diff_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_amcl_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_rb_;

    static geometry_msgs::msg::PoseStamped rigidbody_;
    static geometry_msgs::msg::PoseStamped amcl_;
};

geometry_msgs::msg::PoseStamped MeasurementSubscriber::rigidbody_;
geometry_msgs::msg::PoseStamped MeasurementSubscriber::amcl_;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MeasurementSubscriber>());
  rclcpp::shutdown();
  return 0;
}