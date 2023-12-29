#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using std::placeholders::_1;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "/shelfino0/navigate_to_pose");
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Finished setting up listener");
  }

private:
  geometry_msgs::msg::PoseStamped create_pose(int y)
  {
    geometry_msgs::msg::PoseStamped p_tmp;
    p_tmp.header.stamp = this->get_clock()->now();
    p_tmp.header.frame_id = "map";
    p_tmp.pose.position.x = 0.0;
    p_tmp.pose.position.y = y;
    p_tmp.pose.position.z = 0.0;
    p_tmp.pose.orientation.x = 0.0;
    p_tmp.pose.orientation.y = 0.0;
    p_tmp.pose.orientation.z = 0.0;
    p_tmp.pose.orientation.w = 1.0;
    return p_tmp;
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "msg sending started");
    RCLCPP_INFO(this->get_logger(), "Connect to server");
    if (!this->client_ptr_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    goal_msg.pose.pose.position.x = 2.0;
    goal_msg.pose.pose.position.y = 0.0;
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;
    goal_msg.pose.pose.orientation.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "Sending msg");
    this->client_ptr_->async_send_goal(goal_msg);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}