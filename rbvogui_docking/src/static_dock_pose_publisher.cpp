#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class StaticDockPosePublisher : public rclcpp::Node
{
  public:
    StaticDockPosePublisher()
    : Node("static_dock_publisher"), count_(0)
    {
      auto pose = geometry_msgs::msg::PoseStamped();

      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&StaticDockPosePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto msg = geometry_msgs::msg::PoseStamped();
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "robot_odom";
      msg.pose.position.x = 5.0;
      msg.pose.position.y = 0.0;
      msg.pose.position.z = 0.0;
      msg.pose.orientation.x = 0.4999998;
      msg.pose.orientation.y = -0.4999998;
      msg.pose.orientation.z = -0.4996018;
      msg.pose.orientation.w = 0.5003982;
      publisher_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticDockPosePublisher>());
  rclcpp::shutdown();
  return 0;
}