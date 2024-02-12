#include "Utils.h"

using namespace std;

double get_euclidean_distance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

bool check_collision(double x, double y, Box obstacle)
{
    // points start from bottom left and go clockwise
    if (x >= obstacle.tl.x && x <= obstacle.tr.x && y >= obstacle.bl.y && y <= obstacle.tl.y)
    {
        return true;
    }
    return false;
}