#include <memory>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "RRT.h"
#include "Utils.h"

using namespace std;
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
            "/topic_fp", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        subscription_gates_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/gate_position", 10, std::bind(&MinimalSubscriber::callback, this, std::placeholders::_1));
        subscription_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/shelfino0/amcl_pose", 10, std::bind(&MinimalSubscriber::topic_callback_init_pose, this, _1));
        subscription_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "/obstacles", 10, std::bind(&MinimalSubscriber::topic_callback_obstacles, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Finished setting up listener");
    }

private:
    std::vector<Point2d> plan_path() const
    {
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {

        RCLCPP_INFO(this->get_logger(), "Start path planning");
        RRTNode start(initial_pose_.x, initial_pose_.y, 0);
        RRTNode goal(goal_pose_.x, goal_pose_.y, 0);
        std::vector<double> boundary = {-5, 5};
        // print start goal boundary obstacles list
        RCLCPP_INFO(this->get_logger(), "Start: x: %f, y: %f", start.x, start.y);
        RCLCPP_INFO(this->get_logger(), "Goal: x: %f, y: %f", goal.x, goal.y);
        RCLCPP_INFO(this->get_logger(), "Boundary: min: %f, max: %f", boundary[0], boundary[1]);
        for (auto &obs : obstacleList)
        {
            RCLCPP_INFO(this->get_logger(), "Obstacle: x: %f, y: %f", obs.x, obs.y);
        }

        RRT rrt(start, goal, boundary, obstacleList);
        std::vector<Point2d> RRT_path = rrt.planning();
        RCLCPP_INFO(this->get_logger(), "Path planning done");
        nav_msgs::msg::Path full_path;
        full_path.header.stamp = this->now();
        full_path.header.frame_id = "map";
        std::vector<geometry_msgs::msg::PoseStamped> poses_temp;

        std::ofstream outfile("points.csv");

        // traverse path backwards to get the correct order
        for (auto it = RRT_path.rbegin(); it != RRT_path.rend(); ++it)
        {
            Point2d p = *it;
            geometry_msgs::msg::PoseStamped p_tmp;
            p_tmp.header.stamp = this->now();
            p_tmp.header.frame_id = "map";
            p_tmp.pose.position.x = p.x;
            p_tmp.pose.position.y = p.y;
            p_tmp.pose.position.z = 0.0;
            p_tmp.pose.orientation.x = 0.0;
            p_tmp.pose.orientation.y = 0.0;
            p_tmp.pose.orientation.z = 0.0;
            p_tmp.pose.orientation.w = 1.0;
            outfile << p.x << "," << p.y << ", 0" << std::endl;
            poses_temp.push_back(p_tmp);
        }

        outfile.close();
        system("python3 print.py");

        full_path.poses = poses_temp;
        RCLCPP_INFO(this->get_logger(), "Created path");
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

    void callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        for (const auto &pose : msg->poses)
        {
            goal_pose_.x = pose.position.x;
            goal_pose_.y = pose.position.y;
            if (goal_pose_.x < 0)
                goal_pose_.x += 0.5;
            else
                goal_pose_.x -= 0.5;

            if (goal_pose_.y < 0)
                goal_pose_.y += 0.5;
            else
                goal_pose_.y -= 0.5;

            RCLCPP_INFO(this->get_logger(), "Goal pose - x: %f, y: %f", goal_pose_.x, goal_pose_.y);
        }
    }

    void topic_callback_init_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        initial_pose_.x = msg->pose.pose.position.x;
        initial_pose_.y = msg->pose.pose.position.y;

        // Print the x, y coordinates
        RCLCPP_INFO(get_logger(), "Received pose - x: %f, y: %f", initial_pose_.x, initial_pose_.y);
    }

    void topic_callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        obstacleList.clear();
        for (const auto &obstacle : msg->obstacles)
        {
            Obstacle obs;
            obs.x = obstacle.polygon.points[0].x;
            obs.y = obstacle.polygon.points[0].y;
            obs.radius = obstacle.radius;
            obstacleList.push_back(obs);
        }
    }

    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_gates_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_initial_pose_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_obstacles_;
    Point2d initial_pose_;
    Point2d goal_pose_;
    vector<Obstacle> obstacleList;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSubscriber>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}