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

double get_euclidean_distance(double x0, double y0, double x1, double y1)
{
    double dx = x1 - x0;
    double dy = y1 - y0;
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
    Circle()
    {
        this->x = 0;
        this->y = 0;
        this->radius = 0;
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

Point2d start = make_pair(0, 0);
Point2d goal = make_pair(10, 10);

typedef enum DubinsPathType
{
    LSL = 0,
    LSR = 1,
    RSL = 2,
    RSR = 3,
    RLR = 4,
    LRL = 5
} DubinsPathType;

typedef struct
{
    double timestamp;
    double steering_angle;
} DubinsControl;

typedef struct
{
    double length;
    enum DubinsPathType type;
    DubinsControl controls[3];
} DubinsPath;

double get_arc_length(Point2d start, Point2d end, Circle center, bool is_right_turn)
{
    Point2d vec1, vec2;
    // V1 vector = start - center
    vec1.first = start.first - center.x;
    vec1.second = start.second - center.y;
    // V2 vector = end - center
    vec2.first = end.first - center.x;
    vec2.second = end.second - center.y;

    // Check direction v1 rotated to end up at v2
    double theta = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    if (is_right_turn)
    {
        if (theta > 0)
            theta -= 2.0 * PI; // right turn is a negative rotation
    }
    else
    {
        if (theta < 0)
            theta += 2.0 * PI; // left turn is a positive rotation
    }
    return fabs(MINRADIUS * theta);
}

DubinsControl get_right_turn_control(double arclength)
{
    DubinsControl control;
    control.timestamp = arclength / DELTA;
    control.steering_angle = -1 * MAXSTEER;
    return control;
}

DubinsControl get_left_turn_control(double arclength)
{
    DubinsControl control;
    control.timestamp = arclength / DELTA;
    control.steering_angle = MAXSTEER;
    return control;
}

DubinsControl get_no_turn_control(double arclength)
{
    DubinsControl control;
    control.timestamp = arclength / DELTA;
    control.steering_angle = 0;
    return control;
}

DubinsPath get_RSRPath(vector<pair<Point2d, Point2d>> tangent_points, Circle c1, Circle c2)
{

    if (tangent_points.size() == 0)
    {
        cout << "No tangent points found" << endl;
        return DubinsPath();
    }
    DubinsPath path;
    path.type = RSR;

    Point2d t1 = tangent_points.at(0).first;
    Point2d t2 = tangent_points.at(0).second;
    Point2d start = make_pair(start.first, start.second);
    Point2d end = make_pair(goal.first, goal.second);
    double arclength;

    // right turn
    arclength = get_arc_length(start, t1, c1, true);
    path.controls[0] = get_right_turn_control(arclength);
    path.length = arclength;

    // ahead
    arclength = get_euclidean_distance(t1.first, t1.second, t2.first, t2.second);
    path.controls[1] = get_no_turn_control(arclength);
    path.length += arclength;

    // right turn
    arclength = get_arc_length(t2, end, c2, true);
    path.controls[2] = get_right_turn_control(arclength);
    path.length += arclength;

    return path;
}

DubinsPath get_RSLPath(vector<pair<Point2d, Point2d>> tangent_points, Circle c1, Circle c2)
{

    if (tangent_points.size() == 0)
    {
        cout << "No tangent points found" << endl;
        return DubinsPath();
    }
    DubinsPath path;
    path.type = RSL;

    Point2d t1 = tangent_points.at(2).first;
    Point2d t2 = tangent_points.at(2).second;
    Point2d start = make_pair(start.first, start.second);
    Point2d end = make_pair(goal.first, goal.second);
    double arclength;

    // right turn
    arclength = get_arc_length(start, t1, c1, true);
    path.controls[0] = get_right_turn_control(arclength);
    path.length = arclength;

    // ahead
    arclength = get_euclidean_distance(t1.first, t1.second, t2.first, t2.second);
    path.controls[1] = get_no_turn_control(arclength);
    path.length += arclength;

    // left turn
    arclength = get_arc_length(t2, end, c2, false);
    path.controls[2] = get_left_turn_control(arclength);
    path.length += arclength;

    return path;
}

DubinsPath get_LSLPath(vector<pair<Point2d, Point2d>> tangent_points, Circle c1, Circle c2)
{
    if (tangent_points.size() == 0)
    {
        cout << "No tangent points found" << endl;
        return DubinsPath();
    }
    DubinsPath path;
    path.type = LSL;

    Point2d t1 = tangent_points.at(1).first;
    Point2d t2 = tangent_points.at(1).second;
    Point2d start = make_pair(start.first, start.second);
    Point2d end = make_pair(goal.first, goal.second);
    double arclength;

    // left turn
    arclength = get_arc_length(start, t1, c1, false);
    path.controls[0] = get_left_turn_control(arclength);
    path.length = arclength;

    // ahead
    arclength = get_euclidean_distance(t1.first, t1.second, t2.first, t2.second);
    path.controls[1] = get_no_turn_control(arclength);
    path.length += arclength;

    // left turn
    arclength = get_arc_length(t2, end, c2, false);
    path.controls[2] = get_left_turn_control(arclength);
    path.length += arclength;

    return path;
}

DubinsPath get_LSRPath(vector<pair<Point2d, Point2d>> tangent_points, Circle c1, Circle c2)
{

    if (tangent_points.size() == 0)
    {
        cout << "No tangent points found" << endl;
        return DubinsPath();
    }

    DubinsPath path;
    path.type = LSR;

    Point2d t1 = tangent_points.at(3).first;
    Point2d t2 = tangent_points.at(3).second;
    Point2d start = make_pair(start.first, start.second);
    Point2d end = make_pair(goal.first, goal.second);
    double arclength;

    // left turn
    arclength = get_arc_length(start, t1, c1, false);
    path.controls[0] = get_left_turn_control(arclength);
    path.length = arclength;

    // ahead
    arclength = get_euclidean_distance(t1.first, t1.second, t2.first, t2.second);
    path.controls[1] = get_no_turn_control(arclength);
    path.length += arclength;

    // right turn
    arclength = get_arc_length(t2, end, c2, true);
    path.controls[2] = get_right_turn_control(arclength);
    path.length += arclength;

    return path;
}

void get_CSCPath(Point2d start, Point2d end, double start_theta, double goal_theta)
{
    Circle start_left;
    Circle start_right;
    Circle end_left;
    Circle end_right;

    double theta = start_theta;
    theta += PI / 2.0;
    if (theta > PI)
        theta -= 2.0 * PI;

    start_left.x = start.first + MINRADIUS * cos(theta);
    start_left.y = start.second + MINRADIUS * sin(theta);
    start_left.radius = MINRADIUS;

    theta = start_theta;
    theta -= PI / 2.0;
    if (theta < -PI)
        theta += 2.0 * PI;

    start_right.x = start.first + MINRADIUS * cos(theta);
    start_right.y = start.second + MINRADIUS * sin(theta);
    start_right.radius = MINRADIUS;

    theta = goal_theta;
    theta += PI / 2.0;
    if (theta > PI)
        theta -= 2.0 * PI;

    end_left.x = goal.first + MINRADIUS * cos(theta);
    end_left.y = goal.second + MINRADIUS * sin(theta);
    end_left.radius = MINRADIUS;

    theta = goal_theta;
    theta -= PI / 2.0;
    if (theta < -PI)
        theta += 2.0 * PI;

    end_right.x = goal.first + MINRADIUS * cos(theta);
    end_right.y = goal.second + MINRADIUS * sin(theta);
    end_right.radius = MINRADIUS;

    // print Circles
    cout << "start_left: " << start_left.x << ", " << start_left.y << endl;
    cout << "start_right: " << start_right.x << ", " << start_right.y << endl;
    cout << "end_left: " << end_left.x << ", " << end_left.y << endl;
    cout << "end_right: " << end_right.x << ", " << end_right.y << endl;

    // RSR
    vector<pair<Point2d, Point2d>> tangent_points = create_tangent(start_right, end_right);
    DubinsPath RSR_length = get_RSRPath(tangent_points, start_right, end_right);
    // RSL
    tangent_points = create_tangent(start_right, end_left);
    DubinsPath RSL_length = get_RSLPath(tangent_points, start_right, end_left);
    // LSR
    tangent_points = create_tangent(start_left, end_right);
    DubinsPath LSR_length = get_LSRPath(tangent_points, start_left, end_right);
    // LSL
    tangent_points = create_tangent(start_left, end_left);
    DubinsPath LSL_length = get_LSLPath(tangent_points, start_left, end_left);

    // get the smallest length
    // print lenghts
    cout << "RSR_length: " << RSR_length.length << endl;
    cout << "RSL_length: " << RSL_length.length << endl;
    cout << "LSR_length: " << LSR_length.length << endl;
    cout << "LSL_length: " << LSL_length.length << endl;
    // print shortest DUbinsPath WITH INFO
}

DubinsPath get_RLRPath(Circle c1, Circle c2)
{
    double distance = get_euclidean_distance(c1.x, c1.y, c2.x, c2.y);
    double theta = acos(distance / (4 * MINRADIUS));
    if (distance > 4 * MINRADIUS)
    {
        cout << "No tangent points found" << endl;
        return DubinsPath();
    }
    DubinsPath path;
    path.type = RLR;
    Point2d vec1, vec2;
    vec1.first = c2.x - c1.x;
    vec1.second = c2.y - c1.y;
    double theta1 = atan2(vec1.second, vec1.first);
    // for an RLR trajectory we want to subtract \theta from it to obtain a circle “to the right"
    theta = theta1 - theta;

    Circle c3;
    c3.x = c1.x + 2 * MINRADIUS * cos(theta);
    c3.y = c1.y + 2 * MINRADIUS * sin(theta);
    c3.radius = MINRADIUS;
    cout << "Circle 3: " << c3.x << ", " << c3.y << endl;
    Point2d tangent_point1;
    tangent_point1.first = (c1.x + c3.x) / 2;
    tangent_point1.second = (c1.y + c3.y) / 2;

    vec1.first = start.first - c1.x;
    vec1.second = start.second - c1.y;
    // V2 = end - center
    vec2.first = tangent_point1.first - c1.x;
    vec2.second = tangent_point1.second - c1.y;
    // Check direction v1 rotated to end up at v2
    theta = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    // right turn is a negative rotation
    if (theta > 0)
        theta -= 2.0 * PI;
    double arclength = fabs(MINRADIUS * theta);
    double timesteps = arclength / DELTA;
    DubinsControl control1;
    control1.timestamp = timesteps;
    control1.steering_angle = -1 * MAXSTEER;
    path.controls[0] = control1;

    Point2d tangent_point2;
    tangent_point2.first = (c2.x + c3.x) / 2;
    tangent_point2.second = (c2.y + c3.y) / 2;

    // V1 = start - center
    vec1.first = tangent_point1.first - c3.x;
    vec1.second = tangent_point1.second - c3.y;
    // V2 = end - center
    vec2.first = tangent_point2.first - c3.x;
    vec2.second = tangent_point2.second - c3.y;
    // Check direction v1 rotated to end up at v2
    theta = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    // right turn is a negative rotation
    if (theta < 0)
        theta += 2.0 * PI;
    double arclength1 = fabs(MINRADIUS * theta);
    double timesteps1 = arclength1 / DELTA;
    DubinsControl control2;
    control2.timestamp = timesteps1;
    control2.steering_angle = MAXSTEER;
    path.controls[1] = control2;

    vec1.first = tangent_point2.first - c2.x;
    vec1.second = tangent_point2.second - c2.y;
    // V2 = end - center
    vec2.first = goal.first - c2.x;
    vec2.second = goal.second - c2.y;
    // Check direction v1 rotated to end up at v2
    theta = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    // right turn is a negative rotation
    if (theta > 0)
        theta -= 2.0 * PI;
    double arclength3 = fabs(MINRADIUS * theta);
    double timesteps3 = arclength3 / DELTA;
    DubinsControl control3;
    control3.timestamp = timesteps3;
    control3.steering_angle = -1 * MAXSTEER;
    path.controls[2] = control3;

    // PRINT RESULTS
    cout << "arclength: " << arclength << ", timesteps: " << timesteps << endl;
    cout << "arclength1: " << arclength1 << ", timesteps1: " << timesteps1 << endl;
    cout << "arclength3: " << arclength3 << ", timesteps3: " << timesteps3 << endl;
}

DubinsPath get_LRLPath(Circle c1, Circle c2)
{
    double distance = get_euclidean_distance(c1.x, c1.y, c2.x, c2.y);
    double theta = acos(distance / (4 * MINRADIUS));
    if (distance > 4 * MINRADIUS)
    {
        cout << "No tangent points found" << endl;
        return DubinsPath();
    }
    DubinsPath path;
    path.type = LRL;
    Point2d vec1, vec2;
    vec1.first = c2.x - c1.x;
    vec1.second = c2.y - c1.y;
    double theta1 = atan2(vec1.second, vec1.first);
    // for an LRL trajectory we want to add \theta from it to obtain a circle “to the right"
    theta = theta1 + theta;

    Circle c3;
    c3.x = c1.x + 2 * MINRADIUS * cos(theta);
    c3.y = c1.y + 2 * MINRADIUS * sin(theta);
    c3.radius = MINRADIUS;
    cout << "Circle 3: " << c3.x << ", " << c3.y << endl;
    Point2d tangent_point1;
    tangent_point1.first = (c1.x + c3.x) / 2;
    tangent_point1.second = (c1.y + c3.y) / 2;

    vec1.first = start.first - c1.x;
    vec1.second = start.second - c1.y;
    // V2 = end - center
    vec2.first = tangent_point1.first - c1.x;
    vec2.second = tangent_point1.second - c1.y;
    // Check direction v1 rotated to end up at v2
    theta = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    // right turn is a negative rotation
    if (theta < 0)
        theta += 2.0 * PI;
    double arclength = fabs(MINRADIUS * theta);
    double timesteps = arclength / DELTA;
    DubinsControl control1;
    control1.timestamp = timesteps;
    control1.steering_angle = 1 * MAXSTEER;
    path.controls[0] = control1;

    Point2d tangent_point2;
    tangent_point2.first = (c2.x + c3.x) / 2;
    tangent_point2.second = (c2.y + c3.y) / 2;

    // V1 = start - center
    vec1.first = tangent_point1.first - c3.x;
    vec1.second = tangent_point1.second - c3.y;
    // V2 = end - center
    vec2.first = tangent_point2.first - c3.x;
    vec2.second = tangent_point2.second - c3.y;
    // Check direction v1 rotated to end up at v2
    theta = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    // right turn is a negative rotation
    if (theta > 0)
        theta -= 2.0 * PI;
    double arclength1 = fabs(MINRADIUS * theta);
    double timesteps1 = arclength1 / DELTA;
    DubinsControl control2;
    control2.timestamp = timesteps1;
    control2.steering_angle = -1 * MAXSTEER;
    path.controls[1] = control2;

    vec1.first = tangent_point2.first - c2.x;
    vec1.second = tangent_point2.second - c2.y;
    // V2 = end - center
    vec2.first = goal.first - c2.x;
    vec2.second = goal.second - c2.y;
    // Check direction v1 rotated to end up at v2
    theta = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
    // right turn is a negative rotation
    if (theta < 0)
        theta += 2.0 * PI;
    double arclength3 = fabs(MINRADIUS * theta);
    double timesteps3 = arclength3 / DELTA;
    DubinsControl control3;
    control3.timestamp = timesteps3;
    control3.steering_angle = 1 * MAXSTEER;
    path.controls[2] = control3;

    // PRINT RESULTS
    cout << "arclength: " << arclength << ", timesteps: " << timesteps << endl;
    cout << "arclength1: " << arclength1 << ", timesteps1: " << timesteps1 << endl;
    cout << "arclength3: " << arclength3 << ", timesteps3: " << timesteps3 << endl;
    path.length = (arclength + arclength1 + arclength3);
    return path;
}

void get_CCCPath(Point2d start, Point2d goal, double start_theta, double goal_theta)
{
    Circle start_left;
    Circle start_right;
    Circle end_left;
    Circle end_right;

    double theta = start_theta;
    theta += PI / 2.0;
    if (theta > PI)
        theta -= 2.0 * PI;

    start_left.x = start.first + MINRADIUS * cos(theta);
    start_left.y = start.second + MINRADIUS * sin(theta);
    start_left.radius = MINRADIUS;

    theta = start_theta;
    theta -= PI / 2.0;
    if (theta < -PI)
        theta += 2.0 * PI;

    start_right.x = start.first + MINRADIUS * cos(theta);
    start_right.y = start.second + MINRADIUS * sin(theta);
    start_right.radius = MINRADIUS;

    theta = goal_theta;
    theta += PI / 2.0;
    if (theta > PI)
        theta -= 2.0 * PI;

    end_left.x = goal.first + MINRADIUS * cos(theta);
    end_left.y = goal.second + MINRADIUS * sin(theta);
    end_left.radius = MINRADIUS;

    theta = goal_theta;
    theta -= PI / 2.0;
    if (theta < -PI)
        theta += 2.0 * PI;

    end_right.x = goal.first + MINRADIUS * cos(theta);
    end_right.y = goal.second + MINRADIUS * sin(theta);
    end_right.radius = MINRADIUS;

    // print Circles
    cout << "start_left: " << start_left.x << ", " << start_left.y << endl;
    cout << "start_right: " << start_right.x << ", " << start_right.y << endl;
    cout << "end_left: " << end_left.x << ", " << end_left.y << endl;
    cout << "end_right: " << end_right.x << ", " << end_right.y << endl;

    // RLR
    DubinsPath RLR_length = get_RLRPath(start_right, end_right);
    cout << "RLR_length: " << RLR_length.length << endl;
    // LRL
    DubinsPath LRL_length = get_LRLPath(start_left, end_left);
    cout << "LRL_length: " << LRL_length.length << endl;
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
    new_node.cost = from_node->cost + get_euclidean_distance(from_node->x, from_node->y, goal.first, goal.second);
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
    // double initial_x_ = 0;
    // double initial_y_ = 0;
    // double goal_x_ = std::stod(argv[1]);
    // double goal_y_ = std::stod(argv[2]);
    // RRTNode start(initial_x_, initial_y_, 0);
    // RRTNode goal(goal_x_ goal_y_, 0);
    // std::vector<double> boundary = {-3.1, 3.1};
    // RRT rrt(start, goal, boundary);
    start = make_pair(0, 0);
    double goal_x_ = std::stod(argv[1]);
    double goal_y_ = std::stod(argv[2]);
    goal = make_pair(goal_x_, goal_y_);

    get_CSCPath(start, goal, 0, 0);
    get_CCCPath(start, goal, 0, 0);
}