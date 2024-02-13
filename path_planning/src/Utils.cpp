#include "Utils.h"
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

double get_euclidean_distance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

#include <cstdio> // Include the necessary header file for printf

bool is_collision(double x, double y, Box obstacle)
{
    // points start from bottom left and go clockwise
    //cout << x << " < " << obstacle.tr.x << " && " << x << ">" << obstacle.tl.x << " && " << y << " < " << obstacle.tr.y << " && " << y << " > " << obstacle.br.y << endl;
    //cout << (x < obstacle.tr.x && x > obstacle.tl.x && y < obstacle.tr.y && y > obstacle.br.y) << endl;
    if (x < obstacle.tr.x && x > obstacle.tl.x && y < obstacle.tr.y && y > obstacle.br.y)
    {
        return true;
    }

    return false;
}

bool is_inside_map(double x, double y, Map map)
{
    if (x >= map.bl.x && x <= map.br.x && y >= map.bl.y && y <= map.tl.y)
    {
        return true;
    }
    return false;
}