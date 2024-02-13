#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <vector>

struct Obstacle
{
    double x;
    double y;
    double radius;
};

struct Point2D
{
    double x;
    double y;

    Point2D(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

    Point2D()
    {
        this->x = 0;
        this->y = 0;
    }
};

struct Box
{
    Point2D bl;
    Point2D br;
    Point2D tr;
    Point2D tl;
    double radius;
};

struct Map
{
    Point2D bl;
    Point2D br;
    Point2D tr;
    Point2D tl;
};

struct PointDubins
{
    double x;
    double y;
    double theta;
    PointDubins(double x, double y, double theta)
    {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }
    PointDubins()
    {
        this->x = 0;
        this->y = 0;
        this->theta = 0;
    }
};

double get_euclidean_distance(double x1, double y1, double x2, double y2);

bool check_collision(double x, double y, Box obstacle);

bool check_inside_map(double x, double y, Map map);

#endif // UTILS_H