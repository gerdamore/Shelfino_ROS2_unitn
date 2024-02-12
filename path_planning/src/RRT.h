#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "Utils.h"

#define MAX_ITER 1000
#define GOAL_PROBABILITY 10

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
    RRTNode();
    RRTNode(double x, double y, double yaw);
};

class RRT
{
public:
    RRT(RRTNode start, RRTNode goal, std::vector<double> boundar, std::vector<Obstacle> obstacleList);

    void print_node(int ind, RRTNode node);

    std::vector<Point2d> planning();

private:
    std::vector<Point2d> get_final_path(RRTNode node);

    RRTNode get_shortest_path();

    RRTNode get_path(RRTNode *from_node, RRTNode to_node, int parent_index);

    RRTNode get_random_node();

    int get_closest_index(RRTNode rnd_node);

    bool is_collision(RRTNode node);

    RRTNode start;
    RRTNode end;
    std::vector<RRTNode> node_list;
    double robot_radius;
    double min_rand;
    double max_rand;
    std::vector<Obstacle> obstacleList;
};

#endif // RRT_H