#ifndef UTILS_H
#define UTILS_H

#include <cmath>

struct Obstacle
{
    double x;
    double y;
    double radius;
};

struct Point2d
{
    double x;
    double y;

    Point2d(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

    Point2d()
    {
        this->x = 0;
        this->y = 0;
    }
};

struct Box
{
    Point2d bl;
    Point2d br;
    Point2d tr;
    Point2d tl;
    double radius;
};

struct Map
{
    Point2d bl;
    Point2d br;
    Point2d tr;
    Point2d tl;
};

struct PointDubins
{
    double x;
    double y;
    double theta;
};

double get_euclidean_distance(double x1, double y1, double x2, double y2);

bool check_collision(double x, double y, Box obstacle);

bool check_inside_map(double x, double y, Map map);

#endif // UTILS_H