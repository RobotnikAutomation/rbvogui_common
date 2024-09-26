// Copyright (c) 2024 Open Navigation LLC
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

#include <memory>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class DockPosePublisher : public rclcpp::Node
{
  public:
    DockPosePublisher()
    : Node("dock_pose_publisher")
    {
      subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
      "aruco_markers", 10, std::bind(&DockPosePublisher::detectionCallback, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", 10);

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }

  private:

    void detectionCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
      changeFrameToOdom(msg);
      geometry_msgs::msg::PoseStamped output_pose;
      output_pose.header = msg->header;

      if(msg->poses.size()==4){
        output_pose.pose.position = calculateMidpoint(msg->poses);
        output_pose.pose.orientation = calculateMidOrientation(msg->poses);
      }else if(msg->poses.size() == 3){
        if(msg->marker_ids[0] == msg->marker_ids[1]){
          msg->poses.erase(msg->poses.begin() + 2);
          output_pose.pose.position = calculateMidpoint(msg->poses);
          output_pose.pose.orientation = calculateMidOrientation(msg->poses);
        }else if(msg->marker_ids[0] == msg->marker_ids[2]){
          msg->poses.erase(msg->poses.begin() + 1);
          output_pose.pose.position = calculateMidpoint(msg->poses);
          output_pose.pose.orientation = calculateMidOrientation(msg->poses);
        }else{
          msg->poses.erase(msg->poses.begin());
          output_pose.pose.position = calculateMidpoint(msg->poses);
          output_pose.pose.orientation = calculateMidOrientation(msg->poses);
        }
      }else if(msg->poses.size() == 2){
        if(msg->marker_ids[0] == msg->marker_ids[1]){
          output_pose.pose.position = calculateMidpoint(msg->poses);
          output_pose.pose.orientation = calculateMidOrientation(msg->poses);
        }else{
          output_pose.pose.position = msg->poses[0].position;
          output_pose.pose.orientation = msg->poses[0].orientation;
        }
      }else{
        output_pose.pose.position = msg->poses[0].position;
        output_pose.pose.orientation = msg->poses[0].orientation;
      }

      publisher_->publish(output_pose);

    }

    void changeFrameToOdom(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){

        std::string target_frame = "robot_odom"; // Cambia esto según tu necesidad

        for (size_t i = 0; i < msg->poses.size(); ++i)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = msg->header;
            pose_stamped.pose = msg->poses[i];

            geometry_msgs::msg::PoseStamped transformed_pose_stamped;
            try
            {
                transformed_pose_stamped = tf_buffer_->transform(pose_stamped, target_frame);
                msg->poses[i] = transformed_pose_stamped.pose;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Could not transform pose: %s", ex.what());
            }
        }

        msg->header.frame_id = target_frame;
    }

     geometry_msgs::msg::Point calculateMidpoint(const std::vector<geometry_msgs::msg::Pose>& poses)
    {
        double x_sum = 0.0;
        double y_sum = 0.0;
        double z_sum = 0.0;

        for (const auto& pose : poses)
        {
            x_sum += pose.position.x;
            y_sum += pose.position.y;
            z_sum += pose.position.z;
        }

        geometry_msgs::msg::Point midpoint;
        midpoint.x = x_sum / poses.size();
        midpoint.y = y_sum / poses.size();
        midpoint.z = z_sum / poses.size();

        return midpoint;
    }

    geometry_msgs::msg::Quaternion calculatePerpendicularOrientation(const std::vector<geometry_msgs::msg::Pose>& poses)
    {
        double dx = poses.back().position.x - poses.front().position.x;
        double dy = poses.back().position.y - poses.front().position.y;

        // Calculate the angle perpendicular to the line
        double angle = std::atan2(dy, dx) + M_PI_2;
        RCLCPP_WARN(this->get_logger(), "angle: %f", angle);

        // Convert angle to quaternion
        tf2::Quaternion quat;
        quat.setRPY(M_PI_2, angle, M_PI_2);

        geometry_msgs::msg::Quaternion orientation;
        orientation.x = quat.x();
        orientation.y = quat.y();
        orientation.z = quat.z();
        orientation.w = quat.w();

        return orientation;
    }

    geometry_msgs::msg::Quaternion calculateMidOrientation(const std::vector<geometry_msgs::msg::Pose>& poses)
    {
        double x_sum = 0.0;
        double y_sum = 0.0;
        double z_sum = 0.0;
        double w_sum = 0.0;

        for (const auto& pose : poses)
        {
            x_sum += pose.orientation.x;
            y_sum += pose.orientation.y;
            z_sum += pose.orientation.z;
            w_sum += pose.orientation.w;
        }

        geometry_msgs::msg::Quaternion midpoint;
        midpoint.x = x_sum / poses.size();
        midpoint.y = y_sum / poses.size();
        midpoint.z = z_sum / poses.size();
        midpoint.w = w_sum / poses.size();

        return midpoint;
    }

    int tag_id_;
    bool use_first_detection_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockPosePublisher>());
  rclcpp::shutdown();
  return 0;
}