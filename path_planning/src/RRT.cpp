#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <tuple>
#include <fstream>
#include <random>
#include "RRT.h"
#include "Dubins.h"
#include "Utils.h"
using namespace std;

RRTNode::RRTNode()
{
    PointDubins point(0, 0, 0);
    this->point = point;
    path_x = {};
    path_y = {};
    path_yaw = {};
    cost = 0.0;
    parent = -1;
}
RRTNode::RRTNode(double x, double y, double yaw)
{
    PointDubins point(x, y, yaw);
    this->point = point;
    this->path_x = {};
    this->path_y = {};
    this->path_yaw = {};
    this->cost = 0.0;
    this->parent = -1;
}

RRT::RRT(RRTNode start, RRTNode goal, Map map, std::vector<Box> obstacleList)
{
    this->start = start;
    this->end = goal;
    this->map = map;
    this->robot_radius = 0.5;
    this->obstacleList = obstacleList;
    node_list.push_back(start);
}

void RRT::print_node(int ind, RRTNode node)
{
    std::cout << "ind: " << ind << " x: " << node.point.x << " y: " << node.point.y << " yaw: " << node.point.theta << " cost: " << node.cost << " parent: " << node.parent << std::endl;

    // while (node.parent != -1)
    // {
    //     cout << node.parent << endl;
    //     node = node_list[node.parent];
    // }
    // cout << endl;
}

bool RRT::check_collision(RRTNode node)
{
    for (const auto &obstacle : obstacleList)
    {
        // check collision accross path_x and path_y of node
        for (int i = 0; i < node.path_x.size(); i++)
        {
            if (is_collision(node.path_x[i], node.path_y[i], obstacle) && !is_inside_map(node.path_x[i], node.path_y[i], map))
            {
                return true;
            }
        }
    }
    return false;
}

RRTPath RRT::planning()
{
    for (int i = 0; i < 20 - 1; i++)
    {
        RRTNode rnd = get_random_node();

        int nearest_ind = get_closest_index(rnd);
        RRTNode new_node = get_path(&node_list[nearest_ind], rnd, nearest_ind);

        if (check_collision(new_node))
        {
            continue;
        }
        node_list.push_back(new_node);
    }

    // if possible just have one Dubins path to the goal
    RRTNode Dubins_node = get_path(&start, end, 0);
    // print_node(-1, Dubins_node);
    if (!check_collision(Dubins_node))
    {
        // cout << "Dubins path to goal found!" << endl;
        node_list.push_back(Dubins_node);
    }

    RRTNode shortest = get_shortest_path();
    RRTPath final_path = RRTPath();
    if (shortest.point.x == 0 && shortest.point.y == 0)
    {
        cout << "No path found!" << endl;
        return final_path;
    }

    final_path.path = get_final_path(shortest);
    final_path.cost = shortest.cost;

    return final_path;
}

std::vector<PointDubins> RRT::get_final_path(RRTNode node)
{
    std::vector<PointDubins> path;
    // path.push_back(PointDubins(end.x, end.y, end.yaw));
    path.push_back(node.point);
    for (int i = node.path_x.size() - 1; i >= 0; i--)
    {
        double ix = node.path_x[i];
        double iy = node.path_y[i];
        double itheta = node.path_yaw[i];
        PointDubins point(ix, iy, itheta);
        path.push_back(point);
    }
    RRTNode parent_node = node_list[node.parent];
    int i = 0;
    while (parent_node.parent != -1)
    {
        // print_node(-1, parent_node);
        int input;
        PointDubins point(parent_node.point.x, parent_node.point.y, parent_node.point.theta);
        path.push_back(point);
        for (int i = parent_node.path_x.size() - 1; i >= 0; i--)
        {
            double ix = parent_node.path_x[i];
            double iy = parent_node.path_y[i];
            double itheta = parent_node.path_yaw[i];
            PointDubins point(ix, iy, itheta);
            path.push_back(point);
            // cout << "ix: " << ix << ", iy: " << iy << endl;
        }
        int parent_index = parent_node.parent;
        parent_node.parent = -1;
        parent_node = node_list[parent_index];
    }
    path.push_back(start.point);
    // reverse the order of path
    std::reverse(path.begin(), path.end());

    return path;
}

RRTNode RRT::get_shortest_path()
{
    // cout << end.x << ", " << end.y << endl;
    std::vector<int> goal_indexes;
    for (int i = 1; i < node_list.size(); i++)
    {
        const RRTNode node = node_list[i];
        double distance = get_euclidean_distance(node.point.x, node.point.y, end.point.x, end.point.y);
        if (distance <= 0.2)
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
    // cout << "best_goal_index: " << best_goal_index << endl;
    return node_list[best_goal_index];
}

RRTNode RRT::get_path(RRTNode *from_node, RRTNode to_node, int parent_index)
{
    int sampling_rate = 10;
    std::vector<double> px, py, pyaw;

    // calculate shortest path from node to node with Dubins path
    Point2D start;
    start.x = from_node->point.x;
    start.y = from_node->point.y;
    Point2D goal;
    goal.x = to_node.point.x;
    goal.y = to_node.point.y;

    to_node.point.theta = atan2(goal.y - start.y, goal.x - start.x);

    Dubins dubins = Dubins(from_node->point, to_node.point);
    DubinsPath path = dubins.get_shortest_path();
    std::vector<PointDubins> trajectory = dubins.get_robot_trajectory(path);
    for (const auto &point : trajectory)
    {
        px.push_back(point.x);
        py.push_back(point.y);
        pyaw.push_back(point.theta);
        // print pyaw
        // cout << point.theta << endl;
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
    PointDubins path_end(px.back(), py.back(), pyaw.back());
    new_node.point = path_end;
    new_node.cost = from_node->cost + path.length;
    return new_node;
}

RRTNode RRT::get_random_node()
{
    if (rand() % 100 > GOAL_PROBABILITY)
    {
        std::uniform_real_distribution<> x_dis(map.bl.x, map.br.x);
        std::uniform_real_distribution<> theta_dis(-PI, PI);
        std::uniform_real_distribution<> y_dis(map.bl.y, map.tl.y);

        std::random_device rd;
        std::mt19937 gen(rd()); // Declare and initialize the random number generator
        double x = x_dis(gen);
        double y = y_dis(gen);
        double theta = theta_dis(gen);
        RRTNode node(x, y, theta);
        return node;
    }
    else
    {
        return end;
    }
}

int RRT::get_closest_index(RRTNode rnd_node)
{
    std::vector<double> dlist;
    for (const auto &node : node_list)
    {
        double distance = get_euclidean_distance(node.point.x, node.point.y, rnd_node.point.x, rnd_node.point.y);
        dlist.push_back(distance);
    }

    // search for the smallest element in dlist and return its index
    std::vector<double>::iterator result = std::min_element(std::begin(dlist), std::end(dlist));
    return std::distance(std::begin(dlist), result);
}

std::vector<Obstacle> get_victims(int n)
{
    std::vector<Obstacle> victimList;
    std::uniform_real_distribution<> x_dis(-5, 5);
    std::uniform_real_distribution<> y_dis(-5, 5);
    std::uniform_real_distribution<> size_dis(100, 500);
    std::random_device rd;
    std::mt19937 gen(rd()); // Declare and initialize the random number generator
    int i = 0;
    while (i < n)
    {
        double x = x_dis(gen);
        double y = y_dis(gen);
        double radius = size_dis(gen);
        Obstacle victim;
        victim.x = x;
        victim.y = y;
        victim.radius = radius;
        victimList.push_back(victim);
        i++;
    }
    return victimList;
}

Map get_map(int x, int y)
{
    Map map;
    map.bl = Point2D(-x, -y);
    map.br = Point2D(x, -y);
    map.tr = Point2D(x, y);
    map.tl = Point2D(-x, y);
    return map;
}

std::vector<Box> get_obstacle(int n)
{
    std::vector<Box> obstacleList;
    std::uniform_real_distribution<> x_dis(-5, 5);
    std::uniform_real_distribution<> y_dis(-5, 5);
    std::uniform_real_distribution<> size_dis(0, 0.5);
    std::random_device rd;
    std::mt19937 gen(rd()); // Declare and initialize the random number generator
    int i = 0;
    while (i < n)
    {
        double x = x_dis(gen);
        double y = y_dis(gen);
        double size = size_dis(gen);
        Box obstacle;
        obstacle.bl = Point2D(x - size, y - size);
        obstacle.br = Point2D(x + size, y - size);
        obstacle.tr = Point2D(x + size, y + size);
        obstacle.tl = Point2D(x - size, y + size);
        obstacle.radius = size;
        obstacleList.push_back(obstacle);
        i++;
    }
    return obstacleList;
}

void print_path(RRTPath path)
{
    // print first path point and last and cost only if cost is not INFINITY
    cout << "Path: " << endl;
    if (path.cost != INFINITY)
    {
        cout << "First point: x: " << path.path[0].x << " y: " << path.path[0].y << " theta: " << path.path[0].theta << endl;
        cout << "Last point: x: " << path.path.back().x << " y: " << path.path.back().y << " theta: " << path.path.back().theta << endl;
        cout << "Cost: " << path.cost << endl;
    }
    else
    {
        cout << "No path found!" << endl;
    }
}

// RRTPath get_rrt_path(PointDubins start, PointDubins goal, Map boundary, std::vector<Box> obstacleList)
// {
//     RRTNode s(start.x, start.y, start.theta);
//     RRTNode g(goal.x, goal.y, goal.theta);
//     RRT rrt(s, g, boundary, obstacleList);
//     RRTPath path = rrt.planning();
//     return path;
// }

// int main(int argc, char *argv[])
// {
//     // double initial_x_ = 0;
//     // double initial_y_ = 0;
//     // double goal_x_ = std::stod(argv[1]);
//     // double goal_y_ = std::stod(argv[2]);
//     // double goal_theta_ = atan2(goal_y_ - initial_y_, goal_x_ - initial_x_);
//     // PointDubins start(initial_x_, initial_y_, 0);
//     // PointDubins goal(goal_x_, goal_y_, goal_theta_);

//     // std::vector<Box> obstacleList = get_obstacle(7);
//     // std::vector<Obstacle> victimList = get_victims(1);

//     Map boundary = get_map(-5, 5);
//     std::vector<Box> obstacleList = {};
//     Box obs;
//     Point2D bl, br, tr, tl;
//     // Add Box: bl: x: 3.220435, y: 0.696474, br: x: 3.220435, y: 1.406717, tr: x: 4.162125, y: 1.406717, tl: x: 4.162125, y: 0.696474
//     bl.x = 3.220435;
//     bl.y = 0.696474;
//     br.x = 3.220435;
//     br.y = 1.406717;
//     tr.x = 4.162125;
//     tr.y = 1.406717;
//     tl.x = 4.162125;
//     tl.y = 0.696474;
//     obs.bl = bl;
//     obs.br = br;
//     obs.tr = tr;
//     obs.tl = tl;
//     obs.radius = 0.5;
//     obstacleList.push_back(obs);

//     PointDubins start(0, 0, 0);
//     PointDubins goal(3.99, 1.35, 0);
//     start.theta = atan2(goal.y - start.y, goal.x - start.x);
//     RRTPath path = get_rrt_path(start, goal, boundary, obstacleList);
//     print_path(path);
//     vector<PointDubins> min_path = path.path;
//     // print obstacles to obstacles.csv
//     // // print obstacles
//     std::ofstream outfile_obstacle("obstacles.csv");
//     for (auto &obs : obstacleList)
//     {
//         outfile_obstacle << obs.bl.x << "," << obs.bl.y << "," << obs.br.x << "," << obs.br.y << "," << obs.tr.x << "," << obs.tr.y << "," << obs.tl.x << "," << obs.tl.y << "," << obs.radius << std::endl;
//     }
//     outfile_obstacle.close();
//     // // print victims
//     // for (auto &victim : victimList)
//     // {
//     //     cout << "Victim: x: " << victim.x << ", y: " << victim.y << ", radius: " << victim.radius << endl;
//     // }

//     // vector<PointDubins> min_path = get_best_path(start, goal, boundary, obstacleList, victimList);

//     // double new_theta = 0;
//     std::ofstream outfile("points.csv");

//     for (const auto &point : min_path)
//     {
//         outfile << point.x << "," << point.y << "," << point.theta << std::endl;
//     }

//     outfile.close();
//     system("python3 print.py");
// }