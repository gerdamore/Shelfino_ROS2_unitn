#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std;
using std::placeholders::_1;
using FollowPath = nav2_msgs::action::FollowPath;

typedef std::pair<double, double> Point2d;

class RRTNode
{
public:
    double yaw;
    double x;
    double y;
    double cost;
    std::vector<double> path_x;
    std::vector<double> path_y;
    std::vector<double> path_yaw;
    int parent;

    RRTNode()
    {
        x = 0.0;
        y = 0.0;
        yaw = 0.0;
        cost = 0.0;
        parent = -1;
    }
    RRTNode(double x, double y, double yaw)
    {
        this->x = x;
        this->y = y;
        this->path_x = {};
        this->path_y = {};
        this->path_yaw = {};
        this->yaw = yaw;
        this->cost = 0.0;
        this->parent = -1;
    }

    // node << operator
};

#define MAX_ITER 25
#define GOAL_PROBABILITY 10

class RRT
{
public:
    RRT(RRTNode start, RRTNode goal, std::vector<double> boundary)
    {
        this->start = RRTNode(start.x, start.y, start.yaw);
        this->end = RRTNode(goal.x, goal.y, goal.yaw);
        this->min_rand = boundary[0];
        this->max_rand = boundary[1];
        this->robot_radius = 0.5;
        node_list.push_back(start);
    }

    void print_node(int ind, RRTNode node)
    {
        std::cout << "ind: " << ind << " x: " << node.x << ", y: " << node.y << ", parent: " << node.parent << std::endl;

        // while (node.parent != -1)
        // {
        //     cout << node.parent << endl;
        //     node = node_list[node.parent];
        // }
        cout << endl;
    }

    std::vector<Point2d> planning()
    {
        std::vector<Point2d> path;

        for (int i = 0; i < MAX_ITER - 1; i++)
        {
            RRTNode rnd = get_random_node();
            int nearest_ind = get_closest_index(rnd);
            RRTNode new_node = get_path(&node_list[nearest_ind], rnd, nearest_ind);
            print_node(i, new_node);
            node_list.push_back(new_node);
        }

        RRTNode shortest = get_shortest_path();
        print_node(-1, shortest);
        if (shortest.x == 0 && shortest.y == 0)
        {
            cout << "No path found!" << endl;
            return path;
        }
        return get_final_path(shortest);
    }

private:
    double get_distance(double x, double y, double end_x, double end_y)
    {
        double dx = x - end_x;
        double dy = y - end_y;
        return std::hypot(dx, dy);
    }

    std::vector<Point2d> get_final_path(RRTNode node)
    {
        std::vector<Point2d> path;
        path.push_back(Point2d(end.x, end.y));
        path.push_back(Point2d(node.x, node.y));
        // print_node(-1, node);
        for (int i = node.path_x.size() - 1; i >= 0; i--)
        {
            double ix = node.path_x[i];
            double iy = node.path_y[i];
            path.push_back(Point2d(ix, iy));
        }
        RRTNode parent_node = node_list[node.parent];
        int i = 0;
        while (parent_node.parent != -1)
        {
            // print_node(-1, parent_node);
            int input;
            path.push_back(Point2d(parent_node.x, parent_node.y));
            for (int i = parent_node.path_x.size() - 1; i >= 0; i--)
            {
                double ix = parent_node.path_x[i];
                double iy = parent_node.path_y[i];
                path.push_back(Point2d(ix, iy));
            }
            int parent_index = parent_node.parent;
            parent_node.parent = -1;
            parent_node = node_list[parent_index];
        }
        path.push_back(Point2d(start.x, start.y));
        cout << "Path: " << endl;
        for (int i = 0; i < path.size(); i++)
        {
            cout << "x: " << path[i].first << ", y: " << path[i].second << endl;
        }
        return path;
    }

    RRTNode get_shortest_path()
    {
        // cout << end.x << ", " << end.y << endl;
        std::vector<int> goal_indexes;
        for (int i = 0; i < node_list.size(); i++)
        {
            const RRTNode &node = node_list[i];
            double distance = get_distance(node.x, node.y, end.x, end.y);
            cout << "i: " << i << ", x: " << node.x << ", y: " << node.y << ", distance: " << distance << endl;
            if (distance <= 0.01)
            {
                goal_indexes.push_back(i);
            }
        }

        if (goal_indexes.empty())
        {
            return RRTNode();
        }

        double min_cost = node_list[goal_indexes[0]].cost;
        int best_goal_index = goal_indexes[0];
        for (int i : goal_indexes)
        {
            // cout << "i: " << i << ", cost: " << node_list[i].cost << endl;
            if (node_list[i].cost < min_cost)
            {
                min_cost = node_list[i].cost;
                best_goal_index = i;
            }
        }
        cout << "best_goal_index: " << best_goal_index << endl;
        return node_list[best_goal_index];
    }

    RRTNode get_path(RRTNode *from_node, RRTNode to_node, int parent_index)
    {
        int sampling_rate = 10;
        std::vector<double> px, py, pyaw;
        double goal_x = to_node.x;
        double goal_y = to_node.y;
        double start_x = from_node->x;
        double start_y = from_node->y;
        double step_goal_x = (goal_x - start_x) / sampling_rate;
        double step_goal_y = (goal_y - start_y) / sampling_rate;

        for (int i = 0; i < sampling_rate; i++)
        {
            px.push_back(start_x + step_goal_x * i);
            py.push_back(start_y + step_goal_y * i);
            pyaw.push_back(0);
        }

        RRTNode new_node = RRTNode();
        new_node.path_x = px;
        new_node.path_y = py;
        new_node.path_yaw = pyaw;
        new_node.parent = parent_index;
        new_node.x = goal_x;
        new_node.y = goal_y;
        new_node.yaw = 0;
        new_node.cost = from_node->cost + get_distance(from_node->x, from_node->y, goal_x, goal_y);
        return new_node;
    }

    RRTNode get_random_node()
    {
        if (rand() % 100 > GOAL_PROBABILITY)
        {
            double rnd_x = ((double)rand() / RAND_MAX) * (max_rand - min_rand) + min_rand;
            double rnd_y = ((double)rand() / RAND_MAX) * (max_rand - min_rand) + min_rand;
            double rnd_yaw = ((double)rand() / RAND_MAX) * (2 * M_PI) - M_PI;
            return RRTNode(rnd_x, rnd_y, 0);
        }
        else
        {
            return RRTNode(end.x, end.y, end.yaw);
        }
    }

    int get_closest_index(RRTNode rnd_node)
    {
        std::vector<double> dlist;
        for (const auto &node : node_list)
        {
            double distance = get_distance(node.x, node.y, rnd_node.x, rnd_node.y);
            dlist.push_back(distance);
        }

        std::vector<double>::iterator result = std::min_element(std::begin(dlist), std::end(dlist));
        return std::distance(std::begin(dlist), result);
    }

    RRTNode start;
    RRTNode end;
    std::vector<RRTNode> node_list;
    double robot_radius;
    double min_rand;
    double max_rand;
};

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
        RCLCPP_INFO(this->get_logger(), "Finished setting up listener");
    }

private:
    void plan_path() const
    {
        RRTNode start(initial_x_, initial_y_, 0);
        RRTNode goal(goal_x_, goal_y_, 0);
        std::vector<double> boundary = {-10, 10};
        RRT rrt(start, goal, boundary);
        std::vector<Point2d> path = rrt.planning();
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {

        RCLCPP_INFO(this->get_logger(), "Start path planning");
        plan_path();
        //  nav_msgs::msg::Path full_path;
        //  full_path.header.stamp = this->now();
        //  full_path.header.frame_id = "map";
        //  std::vector<geometry_msgs::msg::PoseStamped> poses_temp;
        //  for (int i = 0; i < 20; i++)
        //  {
        //      geometry_msgs::msg::PoseStamped p_tmp;
        //      p_tmp.header.stamp = this->now();
        //      p_tmp.header.frame_id = "map";
        //      p_tmp.pose.position.x = 0.0;
        //      p_tmp.pose.position.y = 0.1 * i;
        //      p_tmp.pose.position.z = 0.0;
        //      p_tmp.pose.orientation.x = 0.0;
        //      p_tmp.pose.orientation.y = 0.0;
        //      p_tmp.pose.orientation.z = 0.0;
        //      p_tmp.pose.orientation.w = 1.0;
        //      poses_temp.push_back(p_tmp);
        //  }
        //  full_path.poses = poses_temp;
        //  RCLCPP_INFO(this->get_logger(), "Connect to server");
        //  if (!this->client_ptr_->wait_for_action_server())
        //  {
        //      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        //      rclcpp::shutdown();
        //  }

        // auto goal_msg = FollowPath::Goal();
        // goal_msg.path = full_path;
        // goal_msg.controller_id = "FollowPath";
        // RCLCPP_INFO(this->get_logger(), "Sending msg");
        // this->client_ptr_->async_send_goal(goal_msg);
    }

    void callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        for (const auto &pose : msg->poses)
        {
            goal_x_ = pose.position.x;
            MinimalSubscriber::goal_y_ = pose.position.y;
            if (goal_x_ < 0)
                goal_x_ += 0.5;
            else
                goal_x_ -= 0.5;

            if (goal_y_ < 0)
                goal_y_ += 0.5;
            else
                goal_y_ -= 0.5;

            RCLCPP_INFO(this->get_logger(), "Goal pose - x: %f, y: %f", goal_x_, goal_y_);
        }
    }

    void topic_callback_init_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        initial_x_ = msg->pose.pose.position.x;
        initial_y_ = msg->pose.pose.position.y;

        // Print the x, y coordinates
        RCLCPP_INFO(get_logger(), "Received pose - x: %f, y: %f", initial_x_, initial_y_);
    }

    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_gates_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_initial_pose_;
    double goal_x_;
    double goal_y_;
    double initial_x_;
    double initial_y_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSubscriber>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}