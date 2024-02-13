#include "Utils.h"

using namespace std;

double get_euclidean_distance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

#include <cstdio> // Include the necessary header file for printf

bool check_collision(double x, double y, Box obstacle)
{
    // points start from bottom left and go clockwise
    if (x >= obstacle.tl.x && x <= obstacle.tr.x && y >= obstacle.bl.y && y <= obstacle.tl.y)
    {
        printf("%f >= %f && %f <= %f && %f >= %f && %f <= %f\n", x, obstacle.tl.x, x, obstacle.tr.x, y, obstacle.bl.y, y, obstacle.tl.y);
        return true;
    }
    return false;
}

bool check_inside_map(double x, double y, Map map)
{
    if (x >= map.bl.x && x <= map.br.x && y >= map.bl.y && y <= map.tl.y)
    {
        return true;
    }
    return false;
}