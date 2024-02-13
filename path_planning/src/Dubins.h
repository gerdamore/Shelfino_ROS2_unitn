#ifndef DUBINS_H
#define DUBINS_H

#include <iostream>
#include <vector>
#include <cmath>
#include "Utils.h"

#define WHEELBASE 1
#define MINRADIUS 1.1
#define MAXSTEER asin(WHEELBASE / MINRADIUS)
#define DELTA 0.05
#define PI 3.14159265358979323846

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

enum DubinsPathType
{
    RSR,
    RSL,
    LSR,
    LSL,
    RLR,
    LRL
};

struct DubinsControl
{
    double timestamp;
    double steering_angle;
};

struct DubinsPath
{
    DubinsPathType type;
    double length;
    DubinsControl controls[3];
};

class Dubins
{
public:
    Dubins(PointDubins start, PointDubins goal);
    DubinsPath get_shortest_path();
    std::vector<PointDubins> get_robot_trajectory(DubinsPath path);

private:
    PointDubins start;
    PointDubins goal;
    std::vector<std::pair<Point2D, Point2D>> create_tangent(const Circle &c1, const Circle &c2);
    DubinsPath get_RSRPath(const std::vector<std::pair<Point2D, Point2D>> &tangent_points, const Circle &start, const Circle &end);
    DubinsPath get_RSLPath(const std::vector<std::pair<Point2D, Point2D>> &tangent_points, const Circle &start, const Circle &end);
    DubinsPath get_LSRPath(const std::vector<std::pair<Point2D, Point2D>> &tangent_points, const Circle &start, const Circle &end);
    DubinsPath get_LSLPath(const std::vector<std::pair<Point2D, Point2D>> &tangent_points, const Circle &start, const Circle &end);
    DubinsPath get_RLRPath(const Circle &c1, const Circle &c2);
    DubinsPath get_LRLPath(const Circle &c1, const Circle &c2);
    DubinsControl get_right_turn_control(double arc_length);
    DubinsControl get_left_turn_control(double arc_length);
    DubinsControl get_no_turn_control(double arc_length);
    double get_arc_length(Point2D start, Point2D end, Circle c, bool is_right_turn);
    DubinsPath get_CCCPath(Circle start_left, Circle start_right, Circle end_left, Circle end_right);
    DubinsPath get_CSCPath(Circle start_left, Circle start_right, Circle end_left, Circle end_right);
};
#endif // DUBINS_H