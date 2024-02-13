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
#include "geometry_msgs/msg/polygon.hpp"
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
        subscription_victims_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "/victims", 10, std::bind(&MinimalSubscriber::topic_callback_victims, this, std::placeholders::_1));
        subscription_map_ = this->create_subscription<geometry_msgs::msg::Polygon>(
            "/map_borders", 10, std::bind(&MinimalSubscriber::topic_callback_map, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Finished setting up listener");
    }

private:
    std::vector<geometry_msgs::msg::PoseStamped> get_poses_from_rrtpath(vector<PointDubins> path) const
    {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        std::ofstream outfile("points.csv");
        // traverse path backwards to get the correct order
        for (PointDubins p : path)
        {
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
            cout << p.x << "," << p.y << ", 0" << std::endl;
            poses.push_back(p_tmp);
        }

        outfile.close();
        return poses;
    }

    vector<vector<Obstacle>> generate_combinations(const vector<Obstacle> &array) const
    {
        vector<vector<Obstacle>> result;
        size_t n = array.size();
        for (size_t i = 0; i < (1 << n); ++i)
        {
            vector<Obstacle> combination;
            for (size_t j = 0; j < n; ++j)
            {
                if (i & (1 << j))
                {
                    combination.push_back(array[j]);
                }
            }
            result.push_back(combination);
        }
        return result;
    }

    RRTPath get_rrt_path(PointDubins start, PointDubins goal, Map boundary, std::vector<Box> obstacleList) const
    {
        RRTNode s(start.x, start.y, start.theta);
        RRTNode g(goal.x, goal.y, goal.theta);
        RRT rrt(s, g, boundary, obstacleList);
        RRTPath path = rrt.planning();
        return path;
    }

    vector<PointDubins> get_best_path(PointDubins start, PointDubins end, Map map, vector<Box> obstacleList, vector<Obstacle> victimList) const
    {
        double max_ratio = 0;
        std::vector<PointDubins> min_path;
        vector<vector<Obstacle>> combinations = generate_combinations(victimList);
        for (auto &combination : combinations)
        {
            double sum_weight = 0;
            vector<PointDubins> points_trajectory;
            points_trajectory.push_back(start);
            for (auto &victim : combination)
            {
                points_trajectory.push_back(PointDubins(victim.x, victim.y, 0));
                sum_weight += victim.radius;
            }
            points_trajectory.push_back(end);
            std::vector<PointDubins> current_path;
            double sum_cost = 0;
            points_trajectory[0].theta = atan2(points_trajectory[1].y - points_trajectory[0].y, points_trajectory[1].x - points_trajectory[0].x);
            for (int i = 0; i < points_trajectory.size() - 1; i++)
            {
                cout << "Path: " << i << " x: " << points_trajectory[i].x << " y: " << points_trajectory[i].y << " theta: " << points_trajectory[i].theta << " to x: " << points_trajectory[i + 1].x << " y: " << points_trajectory[i + 1].y << " theta: " << points_trajectory[i + 1].theta << endl;
                RRTPath rrt_path = get_rrt_path(points_trajectory[i], points_trajectory[i + 1], map, obstacleList);
                if (rrt_path.cost == INFINITY)
                {
                    sum_cost = INFINITY;
                    break;
                }

                points_trajectory[i + 1].theta = rrt_path.path.end()->theta;
                cout << "Cost: " << rrt_path.cost << endl;
                sum_cost += rrt_path.cost;
                for (auto &p : rrt_path.path)
                {
                    current_path.push_back(p);
                }
            }

            if (sum_cost != INFINITY)
            {
                cout << "Sum cost: " << sum_cost << endl;

                cout << "Ratio: " << sum_weight / sum_cost << endl;
                if (sum_weight / sum_cost > max_ratio)
                {
                    max_ratio = sum_weight / sum_cost;
                    min_path = current_path;
                }
            }
        }
        cout << "Size :" << min_path.size() << std::endl;
        return min_path;
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {

        RRTNode start(initial_pose_.x, initial_pose_.y, 0);
        RRTNode goal(goal_pose_.x, goal_pose_.y, 0);

        // print start goal boundary obstacles list
        RCLCPP_INFO(this->get_logger(), "Start: x: %f, y: %f", start.point.x, start.point.y);
        RCLCPP_INFO(this->get_logger(), "Goal: x: %f, y: %f", goal.point.x, goal.point.y);
        RCLCPP_INFO(this->get_logger(), "Map: bl: x: %f, y: %f, br: x: %f, y: %f, tr: x: %f, y: %f, tl: x: %f, y: %f", map.bl.x, map.bl.y, map.br.x, map.br.y, map.tr.x, map.tr.y, map.tl.x, map.tl.y);
        for (auto &obs : obstacleList)
        {
            RCLCPP_INFO(this->get_logger(), "Obstacle: bl: x: %f, y: %f, br: x: %f, y: %f, tr: x: %f, y: %f, tl: x: %f, y: %f", obs.bl.x, obs.bl.y, obs.br.x, obs.br.y, obs.tr.x, obs.tr.y, obs.tl.x, obs.tl.y);
        }
        for (auto &obs : victimList)
        {
            RCLCPP_INFO(this->get_logger(), "Victim: x: %f, y: %f weight:%f", obs.x, obs.y, obs.radius);
        }

        std::ofstream outfile_obstacle("obstacles.csv");
        for (auto &obs : obstacleList)
        {
            outfile_obstacle << obs.bl.x << "," << obs.bl.y << "," << obs.br.x << "," << obs.br.y << "," << obs.tr.x << "," << obs.tr.y << "," << obs.tl.x << "," << obs.tl.y << "," << obs.radius << std::endl;
        }
        outfile_obstacle.close();

        std::ofstream outfile_victim("victims.csv");
        for (auto &obs : victimList)
        {
            outfile_victim << obs.x << "," << obs.y << std::endl;
        }
        outfile_obstacle.close();

        RCLCPP_INFO(this->get_logger(), "Start path planning");
        vector<PointDubins> RRT_path = get_best_path(start.point, goal.point, map, obstacleList, victimList);

        nav_msgs::msg::Path full_path;
        full_path.header.stamp = this->now();
        full_path.header.frame_id = "map";

        std::vector<geometry_msgs::msg::PoseStamped> poses = get_poses_from_rrtpath(RRT_path);
        cout << "Poses size: " << poses.size() << endl;
        full_path.poses = poses;
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

        system("python3 print.py");
        this->client_ptr_->async_send_goal(goal_msg);
    }

    void callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        for (const auto &pose : msg->poses)
        {
            goal_pose_.x = pose.position.x;
            goal_pose_.y = pose.position.y;

            RCLCPP_INFO(this->get_logger(), "Goal pose - x: %f, y: %f", goal_pose_.x, goal_pose_.y);
        }
    }

    void topic_callback_init_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        initial_pose_.x = msg->pose.pose.position.x;
        initial_pose_.y = msg->pose.pose.position.y;

        RCLCPP_INFO(get_logger(), "Received pose - x: %f, y: %f", initial_pose_.x, initial_pose_.y);
    }

    void topic_callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        obstacleList.clear();
        for (const auto &obstacle : msg->obstacles)
        {
            Box obs;
            Point2D bl, br, tr, tl;
            // points start from bottom left and go clockwise
            bl.x = obstacle.polygon.points[0].x;
            bl.y = obstacle.polygon.points[0].y;
            br.x = obstacle.polygon.points[1].x;
            br.y = obstacle.polygon.points[1].y;
            tr.x = obstacle.polygon.points[2].x;
            tr.y = obstacle.polygon.points[2].y;
            tl.x = obstacle.polygon.points[3].x;
            tl.y = obstacle.polygon.points[3].y;
            obs.bl = bl;
            obs.br = br;
            obs.tr = tr;
            obs.tl = tl;
            obs.radius = obstacle.radius;
            obstacleList.push_back(obs);
        }
    }

    void topic_callback_victims(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        victimList.clear();
        for (const auto &victim : msg->obstacles)
        {
            Obstacle obs;
            obs.x = victim.polygon.points[0].x;
            obs.y = victim.polygon.points[0].y;
            obs.radius = victim.radius; // this will be the weight of he victim
            victimList.push_back(obs);
        }
    }

    void topic_callback_map(const geometry_msgs::msg::Polygon::SharedPtr msg)
    {
        map.bl.x = msg->points[0].x;
        map.bl.y = msg->points[0].y;
        map.br.x = msg->points[1].x;
        map.br.y = msg->points[1].y;
        map.tr.x = msg->points[2].x;
        map.tr.y = msg->points[2].y;
        map.tl.x = msg->points[3].x;
        map.tl.y = msg->points[3].y;
    }

    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_gates_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_initial_pose_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_obstacles_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_victims_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_map_;
    Point2D initial_pose_;
    Point2D goal_pose_;
    vector<Box> obstacleList;
    vector<Obstacle> victimList;
    Map map;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSubscriber>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}