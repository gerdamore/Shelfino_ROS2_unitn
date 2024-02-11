#ifndef UTILS_H
#define UTILS_H

#include <cmath>

double get_euclidean_distance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

#endif // UTILS_H