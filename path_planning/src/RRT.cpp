#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "RRT.h"

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

#define MAX_ITER 200
#define GOAL_SAMPLE_RATE 5

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
    std::cout << "ind: " << ind << " x: " << node.x << ", y: " << node.y << ", parent: " << node.parent << std::endl;

    // while (node.parent != -1)
    // {
    //     cout << node.parent << endl;
    //     node = node_list[node.parent];
    // }
    cout << endl;
}

std::vector<Point2d> RRT::planning()
{
    std::vector<Point2d> path;

    for (int i = 0; i < MAX_ITER - 1; i++)
    {
        RRTNode rnd = get_random_node();
        int nearest_ind = get_closest_index(node_list, rnd);
        RRTNode new_node = get_path(&node_list[nearest_ind], rnd, nearest_ind);
        // print_node(i, new_node);
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

double get_distance(double x, double y, double end_x, double end_y)
{
    double dx = x - end_x;
    double dy = y - end_y;
    return std::hypot(dx, dy);
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
        cout << "ix: " << ix << ", iy: " << iy << endl;
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
            cout << "ix: " << ix << ", iy: " << iy << endl;
        }
        int parent_index = parent_node.parent;
        parent_node.parent = -1;
        parent_node = node_list[parent_index];
    }
    path.push_back(Point2d(start.x, start.y));
    return path;
}

double RRT::calc_dist_to_goal(double x, double y, double end_x, double end_y)
{
    double dx = x - end_x;
    double dy = y - end_y;
    return std::hypot(dx, dy);
}

RRTNode RRT::get_shortest_path()
{
    // cout << end.x << ", " << end.y << endl;
    std::vector<int> goal_indexes;
    for (int i = 0; i < node_list.size(); i++)
    {
        const RRTNode &node = node_list[i];
        double distance = get_distance(node.x, node.y, end.x, end.y);
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

class Circle
{
public:
    double x;
    double y;
    double radius;
    Circle(double x, double y, double radius)
    {
        this->x = x;
        this->y = y;
        this->radius = radius;
    }
};

#define WHEELBASE 1
#define MINRADIUS 1.1
#define MAXSTEER asin(WHEELBASE / MINRADIUS)
#define DELTA 0.05

vector<pair<Point2d, Point2d>> create_tangent(Circle _c1, Circle _c2)
{
    double x1 = _c1.x;
    double y1 = _c1.y;
    double x2 = _c2.x;
    double y2 = _c2.y;
    double r1 = _c1.radius;
    double r2 = _c2.radius;
    double d_sq = pow(x2 - x1, 2) + pow(y2 - y1, 2);
    vector<pair<Point2d, Point2d>> returnVec;

    if (d_sq < (r1 - r2) * (r1 - r2))
    {
        // Circles are contained within each other and not tangent. No tangent lines exist.
        cerr << "Circles are contained within each other and not tangent. No tangent lines exist" << endl;
        return returnVec;
    }

    double d = sqrt(d_sq);
    double vx = (x2 - x1) / d;
    double vy = (y2 - y1) / d;

    // Calculate tangent points for each combination of sign1 and sign2
    // sign1 = +1, sign2 = +1
    for (int sign1 = +1; sign1 >= -1; sign1 -= 2)
    {
        double c = (r1 - sign1 * r2) / d;
        if (c * c > 1.0)
            continue; // want to be subtracting small from large, not adding
        double h = sqrt(max(0.0, 1.0 - c * c));

        for (int sign2 = +1; sign2 >= -1; sign2 -= 2)
        {
            double nx = vx * c - sign2 * h * vy;
            double ny = vy * c + sign2 * h * vx;
            returnVec.push_back(make_pair(make_pair(x1 + r1 * nx, y1 + r1 * ny),
                                          make_pair(x2 + sign1 * r2 * nx, y2 + sign1 * r2 * ny)));
        }
    }

    return returnVec;
}

#define PI 3.14159265

void get_RSRPath(vector<pair<Point2d, Point2d>> tangent_points, Circle c1, Circle c2)
{
    double start_x = 0;
    double start_y = 0;
    double start_theta = 0;

    double goal_x = 3;
    double goal_y = 3;
    double goal_theta = 0;

    if (tangent_points.size() == 0)
    {
        cout << "No tangent points found" << endl;
        return;
    }

    Point2d vec1, vec2;

    // right turn arc length = r1 * theta
    Point2d t1 = tangent_points.at(0).first;
    // V1 = start - center
    vec1.first = start_x - c1.x;
    vec1.second = start_y - c1.y;
    // V2 = end - center
    vec2.first = t1.first - c1.x;
    vec2.second = t1.second - c1.y;
    // Check direction v1 rotated to end up at v2
    double theta = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    // right turn is a negative rotation
    if (theta > 0)
        theta -= 2.0 * PI;
    double arclength = fabs(MINRADIUS * theta);
    double timesteps = arclength / DELTA;

    // ahead arc length
    Point2d t2 = tangent_points.at(0).second;
    double arclength_ahead = get_distance(t1.first, t1.second, t2.first, t2.second);
    double timesteps_ahead = arclength_ahead / DELTA;

    // right turn arc length = r1 * theta
    // V1 = start - center
    vec1.first = t2.first - c2.x;
    vec1.second = t2.second - c2.y;
    // V2 = end - center
    vec2.first = goal_x - c2.x;
    vec2.second = goal_y - c2.y;
    // Check direction v1 rotated to end up at v2
    double theta1 = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    // right turn is a negative rotation
    if (theta1 > 0)
        theta1 -= 2.0 * PI;
    double arclength1 = fabs(MINRADIUS * theta1);
    double timesteps1 = arclength1 / DELTA;

    cout << "arclength: " << arclength << ", timesteps: " << timesteps << endl;
    cout << "arclength_ahead: " << arclength_ahead << ", timesteps_ahead: " << timesteps_ahead << endl;
    cout << "arclength1: " << arclength1 << ", timesteps1: " << timesteps1 << endl;
    cout << arclength1 + arclength + arclength_ahead << endl;
}

void get_RSLPath(vector<pair<Point2d, Point2d>> tangent_points, Circle c1, Circle c2)
{
    double start_x = 0;
    double start_y = 0;
    double start_theta = 0;

    double goal_x = 3;
    double goal_y = 3;
    double goal_theta = 0;

    if (tangent_points.size() == 0)
    {
        cout << "No tangent points found" << endl;
        return;
    }

    Point2d vec1, vec2;

    // right turn arc length = r1 * theta
    Point2d t1 = tangent_points.at(2).first;
    // V1 = start - center
    vec1.first = start_x - c1.x;
    vec1.second = start_y - c1.y;
    // V2 = end - center
    vec2.first = t1.first - c1.x;
    vec2.second = t1.second - c1.y;
    // Check direction v1 rotated to end up at v2
    double theta = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    // right turn is a negative rotation
    if (theta > 0)
        theta -= 2.0 * PI;
    double arclength = fabs(MINRADIUS * theta);
    double timesteps = arclength / DELTA;

    // ahead arc length
    Point2d t2 = tangent_points.at(2).second;
    double arclength_ahead = get_distance(t1.first, t1.second, t2.first, t2.second);
    double timesteps_ahead = arclength_ahead / DELTA;

    // right turn arc length = r1 * theta
    // V1 = start - center
    vec1.first = t2.first - c2.x;
    vec1.second = t2.second - c2.y;
    // V2 = end - center
    vec2.first = goal_x - c2.x;
    vec2.second = goal_y - c2.y;
    // Check direction v1 rotated to end up at v2
    double theta1 = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    // right turn is a negative rotation
    if (theta1 < 0)
        theta1 += 2.0 * PI;
    double arclength1 = fabs(MINRADIUS * theta1);
    double timesteps1 = arclength1 / DELTA;

    cout << "arclength: " << arclength << ", timesteps: " << timesteps << endl;
    cout << "arclength_ahead: " << arclength_ahead << ", timesteps_ahead: " << timesteps_ahead << endl;
    cout << "arclength1: " << arclength1 << ", timesteps1: " << timesteps1 << endl;
    cout << arclength1 + arclength + arclength_ahead << endl;
}

void get_LSLPath(vector<pair<Point2d, Point2d>> tangent_points, Circle c1, Circle c2)
{
    double start_x = 0;
    double start_y = 0;
    double start_theta = 0;

    double goal_x = 3;
    double goal_y = 3;
    double goal_theta = 0;

    if (tangent_points.size() == 0)
    {
        cout << "No tangent points found" << endl;
        return;
    }

    Point2d vec1, vec2;

    // right turn arc length = r1 * theta
    Point2d t1 = tangent_points.at(1).first;
    // V1 = start - center
    vec1.first = start_x - c1.x;
    vec1.second = start_y - c1.y;
    // V2 = end - center
    vec2.first = t1.first - c1.x;
    vec2.second = t1.second - c1.y;
    // Check direction v1 rotated to end up at v2
    double theta = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    // right turn is a negative rotation
    if (theta < 0)
        theta += 2.0 * PI;
    double arclength = fabs(MINRADIUS * theta);
    double timesteps = arclength / DELTA;

    // ahead arc length
    Point2d t2 = tangent_points.at(1).second;
    double arclength_ahead = get_distance(t1.first, t1.second, t2.first, t2.second);
    double timesteps_ahead = arclength_ahead / DELTA;

    // right turn arc length = r1 * theta
    // V1 = start - center
    vec1.first = t2.first - c2.x;
    vec1.second = t2.second - c2.y;
    // V2 = end - center
    vec2.first = goal_x - c2.x;
    vec2.second = goal_y - c2.y;
    // Check direction v1 rotated to end up at v2
    double theta1 = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    // right turn is a negative rotation
    if (theta1 < 0)
        theta1 += 2.0 * PI;
    double arclength1 = fabs(MINRADIUS * theta1);
    double timesteps1 = arclength1 / DELTA;

    cout << "arclength: " << arclength << ", timesteps: " << timesteps << endl;
    cout << "arclength_ahead: " << arclength_ahead << ", timesteps_ahead: " << timesteps_ahead << endl;
    cout << "arclength1: " << arclength1 << ", timesteps1: " << timesteps1 << endl;
    cout << arclength1 + arclength + arclength_ahead << endl;
}

void get_LSRPath(vector<pair<Point2d, Point2d>> tangent_points, Circle c1, Circle c2)
{
    double start_x = 0;
    double start_y = 0;
    double start_theta = 0;

    double goal_x = 3;
    double goal_y = 3;
    double goal_theta = 0;

    if (tangent_points.size() == 0)
    {
        cout << "No tangent points found" << endl;
        return;
    }

    Point2d vec1, vec2;

    // right turn arc length = r1 * theta
    Point2d t1 = tangent_points.at(3).first;
    // V1 = start - center
    vec1.first = start_x - c1.x;
    vec1.second = start_y - c1.y;
    // V2 = end - center
    vec2.first = t1.first - c1.x;
    vec2.second = t1.second - c1.y;
    // Check direction v1 rotated to end up at v2
    double theta = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    // right turn is a negative rotation
    if (theta < 0)
        theta += 2.0 * PI;
    double arclength = fabs(MINRADIUS * theta);
    double timesteps = arclength / DELTA;

    // ahead arc length
    Point2d t2 = tangent_points.at(3).second;
    double arclength_ahead = get_distance(t1.first, t1.second, t2.first, t2.second);
    double timesteps_ahead = arclength_ahead / DELTA;

    // right turn arc length = r1 * theta
    // V1 = start - center
    vec1.first = t2.first - c2.x;
    vec1.second = t2.second - c2.y;
    // V2 = end - center
    vec2.first = goal_x - c2.x;
    vec2.second = goal_y - c2.y;
    // Check direction v1 rotated to end up at v2
    double theta1 = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    // right turn is a negative rotation
    if (theta1 > 0)
        theta1 -= 2.0 * PI;
    double arclength1 = fabs(MINRADIUS * theta1);
    double timesteps1 = arclength1 / DELTA;

    cout << "arclength: " << arclength << ", timesteps: " << timesteps << endl;
    cout << "arclength_ahead: " << arclength_ahead << ", timesteps_ahead: " << timesteps_ahead << endl;
    cout << "arclength1: " << arclength1 << ", timesteps1: " << timesteps1 << endl;
    cout << arclength1 + arclength + arclength_ahead << endl;
}

RRTNode RRT::get_path(RRTNode *from_node, RRTNode to_node, int parent_index)
{
    int sampling_rate = 10;
    std::vector<double> px, py, pyaw;

    // calculate shortest path from node to node with Dubins path

    //
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
    //

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

int initial_x_ = 0;
int initial_y_ = 0;
int goal_x_ = 2;
int goal_y_ = 2;

int main()
{
    RRTNode start(initial_x_, initial_y_, 0);
    RRTNode goal(goal_x_, goal_y_, 0);
    std::vector<double> boundary = {-3.1, 3.1};
    RRT rrt(start, goal, boundary);
    // std::vector<Point2d> path = rrt.planning();
    Circle c1(1.97, -1.1, 1.1);
    Circle c2(3, 1.9, 1.1);
    vector<pair<Point2d, Point2d>> tangent_points = create_tangent(c1, c2);
    get_LSRPath(tangent_points, c1, c2);
}