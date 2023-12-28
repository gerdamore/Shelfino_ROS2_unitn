#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using std::placeholders::_1;
using FollowPath = nav2_msgs::action::FollowPath;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("minimal_subscriber")
    {
        this->initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "shelfino0/amcl_pose", 10, std::bind(&MinimalSubscriber::initial_pose_cb, this, std::placeholders::_1));
        client_ptr_ = rclcpp_action::create_client<FollowPath>(this, "/shelfino0/follow_path");
        RCLCPP_INFO(this->get_logger(), "Finished setting up listener");
    }
    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;

private:
    void initial_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
};

void MinimalSubscriber::initial_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
}

geometry_msgs::msg::PoseStamped create_pose(auto node, int y)
{
    geometry_msgs::msg::PoseStamped p_tmp;
    p_tmp.header.stamp = node->get_clock()->now();
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSubscriber>();

    RCLCPP_INFO(node->get_logger(), "msg sending started");
    nav_msgs::msg::Path full_path;
    full_path.header.stamp = node->get_clock()->now();
    full_path.header.frame_id = "map";
    std::vector<geometry_msgs::msg::PoseStamped> poses_temp;
    for (int i = 0; i < 100; i++)
    {
        poses_temp.push_back(create_pose(node, 0.1 * i));
    }
    full_path.poses = poses_temp;
    RCLCPP_INFO(node->get_logger(), "Connect to server");
    if (!node->client_ptr_->wait_for_action_server())
    {
        RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = FollowPath::Goal();
    goal_msg.path = full_path;
    goal_msg.controller_id = "FollowPath";
    RCLCPP_INFO(node->get_logger(), "Sending msg");
    node->client_ptr_->async_send_goal(goal_msg);

    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}