#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "Utils.h"

#define MAX_ITER 300
#define GOAL_PROBABILITY 10

struct RRTPath
{
    double cost;
    std::vector<PointDubins> path;

    RRTPath()
    {
        cost = INFINITY;
        path = {};
    }
};

class RRTNode
{
public:
    PointDubins point;
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
    RRT(RRTNode start, RRTNode goal, Map map, std::vector<Box> obstacleList);

    void print_node(int ind, RRTNode node);

    RRTPath planning();

private:
    std::vector<PointDubins> get_final_path(RRTNode node);

    RRTNode get_shortest_path();

    RRTNode get_path(RRTNode *from_node, RRTNode to_node, int parent_index);

    RRTNode get_random_node();

    int get_closest_index(RRTNode rnd_node);

    bool is_collision(RRTNode node);

    RRTNode start;
    RRTNode end;
    std::vector<RRTNode> node_list;
    double robot_radius;
    Map map;
    std::vector<Box> obstacleList;
};

#endif // RRT_H