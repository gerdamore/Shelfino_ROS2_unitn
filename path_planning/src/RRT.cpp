#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include "RRT.h"
#include "Dubins.h"

using namespace std;

RRTNode::RRTNode()
{
    x = 0.0;
    y = 0.0;
    yaw = 0.0;
    cost = 0.0;
    parent = -1;
}
RRTNode::RRTNode(double x, double y, double yaw)
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

RRT::RRT(RRTNode start, RRTNode goal, std::vector<double> boundary)
{
    this->start = RRTNode(start.x, start.y, start.yaw);
    this->end = RRTNode(goal.x, goal.y, goal.yaw);
    this->min_rand = boundary[0];
    this->max_rand = boundary[1];
    this->robot_radius = 0.5;
    node_list.push_back(start);
}

void RRT::print_node(int ind, RRTNode node)
{
    std::cout << "ind: " << ind << " x: " << node.x << ", y: " << node.y << ", parent: " << node.parent << ", cost: " << node.cost << endl;

    // while (node.parent != -1)
    // {
    //     cout << node.parent << endl;
    //     node = node_list[node.parent];
    // }
    // cout << endl;
}

std::vector<Point2d> RRT::planning()
{
    std::vector<Point2d> path;

    for (int i = 0; i < 20 - 1; i++)
    {
        RRTNode rnd = get_random_node();
        int nearest_ind = get_closest_index(node_list, rnd);
        RRTNode new_node = get_path(&node_list[nearest_ind], rnd, nearest_ind);
        print_node(i, new_node);
        node_list.push_back(new_node);
    }

    // if possible just have one Dubins path to the goal
    RRTNode Dubins_node = get_path(&start, end, 0);
    print_node(-1, Dubins_node);
    node_list.push_back(Dubins_node);

    RRTNode shortest = get_shortest_path();
    print_node(-1, shortest);
    if (shortest.x == 0 && shortest.y == 0)
    {
        cout << "No path found!" << endl;
        return path;
    }
    return get_final_path(shortest);
}

std::vector<Point2d> RRT::get_final_path(RRTNode node)
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
        // cout << "ix: " << ix << ", iy: " << iy << endl;
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
            // cout << "ix: " << ix << ", iy: " << iy << endl;
        }
        int parent_index = parent_node.parent;
        parent_node.parent = -1;
        parent_node = node_list[parent_index];
    }
    path.push_back(Point2d(start.x, start.y));
    return path;
}

RRTNode RRT::get_shortest_path()
{
    // cout << end.x << ", " << end.y << endl;
    std::vector<int> goal_indexes;
    for (int i = 0; i < node_list.size(); i++)
    {
        const RRTNode &node = node_list[i];
        double distance = get_euclidean_distance(node.x, node.y, end.x, end.y);
        // cout << "i: " << i << ", x: " << node.x << ", y: " << node.y << ", distance: " << distance << endl;
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

RRTNode RRT::get_path(RRTNode *from_node, RRTNode to_node, int parent_index)
{
    int sampling_rate = 10;
    std::vector<double> px, py, pyaw;

    // calculate shortest path from node to node with Dubins path
    Point2d start;
    start.x = from_node->x;
    start.y = from_node->y;
    Point2d goal;
    goal.x = to_node.x;
    goal.y = to_node.y;
    double start_theta = from_node->yaw;
    double goal_theta = atan2(goal.y - start.y, goal.x - start.x);

    Dubins dubins = Dubins(start, goal, start_theta, goal_theta);
    DubinsPath path = dubins.get_shortest_path();
    std::vector<PointDubins> trajectory = dubins.get_robot_trajectory(path);
    for (const auto &point : trajectory)
    {
        px.push_back(point.x);
        py.push_back(point.y);
        pyaw.push_back(point.theta);
    }
    //
    //
    // double goal_x = to_node.x;
    // double goal_y = to_node.y;
    // double start_x = from_node->x;
    // double start_y = from_node->y;
    // double step_goal_x = (goal_x - start_x) / sampling_rate;
    // double step_goal_y = (goal_y - start_y) / sampling_rate;

    // for (int i = 0; i < sampling_rate; i++)
    // {
    //     px.push_back(start_x + step_goal_x * i);
    //     py.push_back(start_y + step_goal_y * i);
    //     pyaw.push_back(0);
    // }
    //

    RRTNode new_node = RRTNode();
    new_node.path_x = px;
    new_node.path_y = py;
    new_node.path_yaw = pyaw;
    new_node.parent = parent_index;
    new_node.x = to_node.x;
    new_node.y = to_node.y;
    new_node.yaw = goal_theta;
    new_node.cost = from_node->cost + path.length;
    return new_node;
}

RRTNode RRT::get_random_node()
{
    if (rand() % 100 > 10)
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

int RRT::get_closest_index(std::vector<RRTNode> node_list, RRTNode rnd_node)
{
    std::vector<double> dlist;
    for (const auto &node : node_list)
    {
        double distance = pow(node.x - rnd_node.x, 2) + pow(node.y - rnd_node.y, 2);
        dlist.push_back(distance);
    }

    // search for the smallest element in dlist and return its index
    std::vector<double>::iterator result = std::min_element(std::begin(dlist), std::end(dlist));
    return std::distance(std::begin(dlist), result);
}

int main(int argc, char *argv[])
{
    double initial_x_ = 0;
    double initial_y_ = 0;
    double goal_x_ = std::stod(argv[1]);
    double goal_y_ = std::stod(argv[2]);

    RRTNode start(initial_x_, initial_y_, 0);
    RRTNode goal(goal_x_, goal_y_, 0);
    std::vector<double> boundary = {-3.1, 3.1};
    RRT rrt(start, goal, boundary);
    std::vector<Point2d> path = rrt.planning();
    std::ofstream outfile("points.csv");
    for (const auto &point : path)
    {
        outfile << point.x << "," << point.y << ", 0" << std::endl;
    }
    outfile.close();
    system("python3 print.py");
}