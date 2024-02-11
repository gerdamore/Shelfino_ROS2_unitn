#include "Dubins.h"
#include "Utils.h"
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

Dubins::Dubins(Point2d start, Point2d goal, double start_theta, double goal_theta)
{
    this->start = start;
    this->goal = goal;
    this->start_theta = start_theta;
    this->goal_theta = goal_theta;
}

vector<pair<Point2d, Point2d>> Dubins::create_tangent(const Circle &c1, const Circle &c2)
{
    double x1 = c1.x;
    double y1 = c1.y;
    double x2 = c2.x;
    double y2 = c2.y;
    double r1 = c1.radius;
    double r2 = c2.radius;
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
            Point2d tangent_point1;
            tangent_point1.x = x1 + r1 * nx;
            tangent_point1.y = y1 + r1 * ny;
            Point2d tangent_point2;
            tangent_point2.x = x2 + sign1 * r2 * nx;
            tangent_point2.y = y2 + sign1 * r2 * ny;
            returnVec.push_back(std::make_pair(tangent_point1, tangent_point2));
        }
    }

    return returnVec;
}

DubinsPath Dubins::get_shortest_path()
{
    Circle start_left;
    Circle start_right;
    Circle end_left;
    Circle end_right;

    double theta = start_theta;
    theta += PI / 2.0;
    if (theta > PI)
        theta -= 2.0 * PI;

    start_left.x = start.x + MINRADIUS * cos(theta);
    start_left.y = start.y + MINRADIUS * sin(theta);
    start_left.radius = MINRADIUS;

    theta = start_theta;
    theta -= PI / 2.0;
    if (theta < -PI)
        theta += 2.0 * PI;

    start_right.x = start.x + MINRADIUS * cos(theta);
    start_right.y = start.y + MINRADIUS * sin(theta);
    start_right.radius = MINRADIUS;

    theta = goal_theta;
    theta += PI / 2.0;
    if (theta > PI)
        theta -= 2.0 * PI;

    end_left.x = goal.x + MINRADIUS * cos(theta);
    end_left.y = goal.y + MINRADIUS * sin(theta);
    end_left.radius = MINRADIUS;

    theta = goal_theta;
    theta -= PI / 2.0;
    if (theta < -PI)
        theta += 2.0 * PI;

    end_right.x = goal.x + MINRADIUS * cos(theta);
    end_right.y = goal.y + MINRADIUS * sin(theta);
    end_right.radius = MINRADIUS;
    get_CSCPath(start_left, start_right, end_left, end_right);
    get_CCCPath(start_left, start_right, end_left, end_right);
}

double Dubins::get_arc_length(Point2d start, Point2d end, Circle center, bool is_right_turn)
{
    Point2d vec1, vec2;
    // V1 vector = start - center
    vec1.x = start.x - center.x;
    vec1.y = start.y - center.y;
    // V2 vector = end - center
    vec2.x = end.x - center.x;
    vec2.y = end.y - center.y;

    // Check direction v1 rotated to end up at v2
    double theta = atan2(vec2.y, vec2.x) - atan2(vec1.y, vec1.x);
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

DubinsControl Dubins::get_right_turn_control(double arclength)
{
    DubinsControl control;
    control.timestamp = arclength / DELTA;
    control.steering_angle = -1 * MAXSTEER;
    return control;
}

DubinsControl Dubins::get_left_turn_control(double arclength)
{
    DubinsControl control;
    control.timestamp = arclength / DELTA;
    control.steering_angle = MAXSTEER;
    return control;
}

DubinsControl Dubins::get_no_turn_control(double arclength)
{
    DubinsControl control;
    control.timestamp = arclength / DELTA;
    control.steering_angle = 0;
    return control;
}

DubinsPath Dubins::get_RSRPath(const vector<pair<Point2d, Point2d>> &tangent_points, const Circle &c1, const Circle &c2)
{

    if (tangent_points.size() == 0)
    {
        std::cout << "No tangent points found" << std::endl;
        DubinsPath path;
        path.length = INFINITY;
        return path;
    }
    DubinsPath path;
    path.type = RSR;

    Point2d t1 = tangent_points.at(0).first;
    Point2d t2 = tangent_points.at(0).second;
    double arclength;

    // right turn
    arclength = get_arc_length(start, t1, c1, true);
    path.controls[0] = get_right_turn_control(arclength);
    path.length = arclength;

    // ahead
    arclength = get_euclidean_distance(t1.x, t1.y, t2.x, t2.y);
    path.controls[1] = get_no_turn_control(arclength);
    path.length += arclength;

    // right turn
    arclength = get_arc_length(t2, goal, c2, true);
    path.controls[2] = get_right_turn_control(arclength);
    path.length += arclength;

    return path;
}

DubinsPath Dubins::get_RSLPath(const vector<pair<Point2d, Point2d>> &tangent_points, const Circle &c1, const Circle &c2)
{

    if (tangent_points.size() < 3)
    {
        std::cout << "No tangent points found" << endl;
        DubinsPath path;
        path.length = INFINITY;
        return path;
    }
    DubinsPath path;
    path.type = RSL;

    Point2d t1 = tangent_points.at(2).first;
    Point2d t2 = tangent_points.at(2).second;
    double arclength;

    // right turn
    arclength = get_arc_length(start, t1, c1, true);
    path.controls[0] = get_right_turn_control(arclength);
    path.length = arclength;

    // ahead
    arclength = get_euclidean_distance(t1.x, t1.y, t2.x, t2.y);
    path.controls[1] = get_no_turn_control(arclength);
    path.length += arclength;

    // left turn
    arclength = get_arc_length(t2, goal, c2, false);
    path.controls[2] = get_left_turn_control(arclength);
    path.length += arclength;

    return path;
}

DubinsPath Dubins::get_LSLPath(const vector<pair<Point2d, Point2d>> &tangent_points, const Circle &c1, const Circle &c2)
{
    if (tangent_points.size() < 2)
    {
        std::cout << "No tangent points found" << endl;
        DubinsPath path;
        path.length = INFINITY;
        return path;
    }
    DubinsPath path;
    path.type = LSL;

    Point2d t1 = tangent_points.at(1).first;
    Point2d t2 = tangent_points.at(1).second;
    double arclength;

    // left turn
    arclength = get_arc_length(start, t1, c1, false);
    path.controls[0] = get_left_turn_control(arclength);
    path.length = arclength;

    // ahead
    arclength = get_euclidean_distance(t1.x, t1.y, t2.x, t2.y);
    path.controls[1] = get_no_turn_control(arclength);
    path.length += arclength;

    // left turn
    arclength = get_arc_length(t2, goal, c2, false);
    path.controls[2] = get_left_turn_control(arclength);
    path.length += arclength;

    return path;
}

DubinsPath Dubins::get_LSRPath(const vector<pair<Point2d, Point2d>> &tangent_points, const Circle &c1, const Circle &c2)
{

    if (tangent_points.size() < 4)
    {
        std::cout << "No tangent points found" << endl;
        DubinsPath path;
        path.length = INFINITY;
        return path;
    }

    DubinsPath path;
    path.type = LSR;

    Point2d t1 = tangent_points.at(3).first;
    Point2d t2 = tangent_points.at(3).second;
    double arclength;

    // left turn
    arclength = get_arc_length(start, t1, c1, false);
    path.controls[0] = get_left_turn_control(arclength);
    path.length = arclength;

    // ahead
    arclength = get_euclidean_distance(t1.x, t1.y, t2.x, t2.y);
    path.controls[1] = get_no_turn_control(arclength);
    path.length += arclength;

    // right turn
    arclength = get_arc_length(t2, goal, c2, true);
    path.controls[2] = get_right_turn_control(arclength);
    path.length += arclength;

    return path;
}

void Dubins::get_CSCPath(Circle start_left, Circle start_right, Circle end_left, Circle end_right)
{

    // print Circles
    // std::cout << "start_left: " << start_left.x << ", " << start_left.y << endl;
    // std::cout << "start_right: " << start_right.x << ", " << start_right.y << endl;
    // std::cout << "end_left: " << end_left.x << ", " << end_left.y << endl;
    // std::cout << "end_right: " << end_right.x << ", " << end_right.y << endl;

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
    std::cout << "RSR_length: " << RSR_length.length << endl;
    std::cout << "LSL_length: " << LSL_length.length << endl;
    std::cout << "RSL_length: " << RSL_length.length << endl;
    std::cout << "LSR_length: " << LSR_length.length << endl;

    // print shortest DUbinsPath WITH INFO
}

DubinsPath Dubins::get_RLRPath(const Circle &c1, const Circle &c2)
{
    double distance = get_euclidean_distance(c1.x, c1.y, c2.x, c2.y);
    double theta = acos(distance / (4 * MINRADIUS));
    if (distance > 4 * MINRADIUS)
    {
        std::cout << "No tangent points found" << endl;
        DubinsPath path;
        path.length = INFINITY;
        return path;
    }
    DubinsPath path;
    path.type = RLR;

    Point2d vec1, vec2;
    vec1.x = c2.x - c1.x;
    vec1.y = c2.y - c1.y;
    double theta1 = atan2(vec1.y, vec1.x);
    // for an RLR trajectory we want to subtract \theta from it to obtain a circle “to the right"
    theta = theta1 - theta;

    Circle c3;
    c3.x = c1.x + 2 * MINRADIUS * cos(theta);
    c3.y = c1.y + 2 * MINRADIUS * sin(theta);
    c3.radius = MINRADIUS;

    Point2d tangent_point1;
    tangent_point1.x = (c1.x + c3.x) / 2;
    tangent_point1.y = (c1.y + c3.y) / 2;

    Point2d tangent_point2;
    tangent_point2.x = (c2.x + c3.x) / 2;
    tangent_point2.y = (c2.y + c3.y) / 2;

    double arclength;

    // right turn
    arclength = get_arc_length(start, tangent_point1, c1, true);
    path.controls[0] = get_right_turn_control(arclength);
    path.length = arclength;

    // left turn
    arclength = get_arc_length(tangent_point1, tangent_point2, c3, false);
    path.controls[1] = get_left_turn_control(arclength);
    path.length += arclength;

    // right turn
    arclength = get_arc_length(tangent_point2, goal, c2, true);
    path.controls[2] = get_right_turn_control(arclength);
    path.length += arclength;

    // PRINT RESULTS
    // std::cout << "arclength: " << arclength << ", timesteps: " << timesteps << endl;
    // std::cout << "arclength1: " << arclength1 << ", timesteps1: " << timesteps1 << endl;
    // std::cout << "arclength3: " << arclength3 << ", timesteps3: " << timesteps3 << endl;
    return path;
}

DubinsPath Dubins::get_LRLPath(const Circle &c1, const Circle &c2)
{
    double distance = get_euclidean_distance(c1.x, c1.y, c2.x, c2.y);
    double theta = acos(distance / (4 * MINRADIUS));
    if (distance > 4 * MINRADIUS)
    {
        std::cout << "No tangent points found" << endl;
        DubinsPath path;
        path.length = INFINITY;
        return path;
    }
    DubinsPath path;
    path.type = LRL;
    Point2d vec1, vec2;
    vec1.x = c2.x - c1.x;
    vec1.y = c2.y - c1.y;
    double theta1 = atan2(vec1.y, vec1.x);
    // for an LRL trajectory we want to add \theta from it to obtain a circle “to the right"
    theta = theta1 + theta;

    Circle c3;
    c3.x = c1.x + 2 * MINRADIUS * cos(theta);
    c3.y = c1.y + 2 * MINRADIUS * sin(theta);
    c3.radius = MINRADIUS;

    Point2d tangent_point1;
    tangent_point1.x = (c1.x + c3.x) / 2;
    tangent_point1.y = (c1.y + c3.y) / 2;

    Point2d tangent_point2;
    tangent_point2.x = (c2.x + c3.x) / 2;
    tangent_point2.y = (c2.y + c3.y) / 2;

    double arclength;

    // left turn
    arclength = get_arc_length(start, tangent_point1, c1, false);
    path.controls[0] = get_left_turn_control(arclength);
    path.length = arclength;

    // right turn
    arclength = get_arc_length(tangent_point1, tangent_point2, c3, true);
    path.controls[1] = get_right_turn_control(arclength);
    path.length += arclength;

    // left turn
    arclength = get_arc_length(tangent_point2, goal, c2, false);
    path.controls[2] = get_left_turn_control(arclength);
    path.length += arclength;

    // PRINT RESULTS
    // std::cout << "arclength: " << arclength << ", timesteps: " << timesteps << endl;
    // std::cout << "arclength1: " << arclength1 << ", timesteps1: " << timesteps1 << endl;
    // std::cout << "arclength3: " << arclength3 << ", timesteps3: " << timesteps3 << endl;

    return path;
}

void Dubins::get_CCCPath(Circle start_left, Circle start_right, Circle end_left, Circle end_right)
{

    // print Circles
    // std::cout << "start_left: " << start_left.x << ", " << start_left.y << endl;
    // std::cout << "start_right: " << start_right.x << ", " << start_right.y << endl;
    // std::cout << "end_left: " << end_left.x << ", " << end_left.y << endl;
    // std::cout << "end_right: " << end_right.x << ", " << end_right.y << endl;

    // RLR
    DubinsPath RLR_length = get_RLRPath(start_right, end_right);
    std::cout << "RLR_length: " << RLR_length.length << endl;
    // LRL
    DubinsPath LRL_length = get_LRLPath(start_left, end_left);
    std::cout << "LRL_length: " << LRL_length.length << endl;
}

int main(int argc, char *argv[])
{
    Point2d start;
    start.x = 0;
    start.y = 0;
    double goal_x_ = std::stod(argv[1]);
    double goal_y_ = std::stod(argv[2]);
    Point2d goal;
    goal.x = goal_x_;
    goal.y = goal_y_;
    double start_theta = 0;
    double goal_theta = 0;
    Dubins dubins(start, goal, start_theta, goal_theta);
    dubins.get_shortest_path();
}