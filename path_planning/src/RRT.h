#ifndef GYAK_H
#define GYAK_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#define MAX_ITER 1000
#define GOAL_PROBABILITY 5

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

typedef std::pair<double, double> Point2d;

class RRT
{
public:
    RRT(RRTNode start, RRTNode goal, std::vector<double> boundary);

    void print_node(int ind, RRTNode node);

    std::vector<Point2d> planning();

private:
    std::vector<Point2d> get_final_path(RRTNode node);

    RRTNode get_shortest_path();

    RRTNode get_path(RRTNode *from_node, RRTNode to_node, int parent_index);

    RRTNode get_random_node();

    int get_closest_index(std::vector<RRTNode> node_list, RRTNode rnd_node);

    RRTNode start;
    RRTNode end;
    std::vector<RRTNode> node_list;
    double robot_radius;
    double min_rand;
    double max_rand;
};

#endif // GYAK_H