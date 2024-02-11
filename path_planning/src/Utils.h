#ifndef UTILS_H
#define UTILS_H

#include <cmath>

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

struct PointDubins
{
    double x;
    double y;
    double theta;
};

double get_euclidean_distance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

#endif // UTILS_H