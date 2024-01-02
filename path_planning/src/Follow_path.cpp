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
        client_ptr_ = rclcpp_action::create_client<FollowPath>(this, "/shelfino0/follow_path");
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic_fp", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Finished setting up listener");
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {

        RCLCPP_INFO(this->get_logger(), "msg sending started");
        nav_msgs::msg::Path full_path;
        full_path.header.stamp = this->now();
        full_path.header.frame_id = "map";
        std::vector<geometry_msgs::msg::PoseStamped> poses_temp;
        for (int i = 0; i < 20; i++)
        {
            geometry_msgs::msg::PoseStamped p_tmp;
            p_tmp.header.stamp = this->now();
            p_tmp.header.frame_id = "map";
            p_tmp.pose.position.x = 0.0;
            p_tmp.pose.position.y = 0.1 * i;
            p_tmp.pose.position.z = 0.0;
            p_tmp.pose.orientation.x = 0.0;
            p_tmp.pose.orientation.y = 0.0;
            p_tmp.pose.orientation.z = 0.0;
            p_tmp.pose.orientation.w = 1.0;
            poses_temp.push_back(p_tmp);
        }
        full_path.poses = poses_temp;
        RCLCPP_INFO(this->get_logger(), "Connect to server");
        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = FollowPath::Goal();
        goal_msg.path = full_path;
        goal_msg.controller_id = "FollowPath";
        RCLCPP_INFO(this->get_logger(), "Sending msg");
        this->client_ptr_->async_send_goal(goal_msg);
    }

    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSubscriber>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}