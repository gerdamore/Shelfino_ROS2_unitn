#include "Dubins.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <fstream>
#include <cstdlib>

using namespace std;

Dubins::Dubins(PointDubins start, PointDubins goal)
{
    this->start = start;
    this->goal = goal;
}

vector<pair<Point2D, Point2D>> Dubins::create_tangent(const Circle &c1, const Circle &c2)
{
    double x1 = c1.x;
    double y1 = c1.y;
    double x2 = c2.x;
    double y2 = c2.y;
    double r1 = c1.radius;
    double r2 = c2.radius;
    double d_sq = pow(x2 - x1, 2) + pow(y2 - y1, 2);
    vector<pair<Point2D, Point2D>> returnVec;

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
            Point2D tangent_point1;
            tangent_point1.x = x1 + r1 * nx;
            tangent_point1.y = y1 + r1 * ny;
            Point2D tangent_point2;
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

    double theta = start.theta;
    theta += PI / 2.0;
    if (theta > PI)
        theta -= 2.0 * PI;

    start_left.x = start.x + MINRADIUS * cos(theta);
    start_left.y = start.y + MINRADIUS * sin(theta);
    start_left.radius = MINRADIUS;

    theta = start.theta;
    theta -= PI / 2.0;
    if (theta < -PI)
        theta += 2.0 * PI;

    start_right.x = start.x + MINRADIUS * cos(theta);
    start_right.y = start.y + MINRADIUS * sin(theta);
    start_right.radius = MINRADIUS;

    theta = goal.theta;
    theta += PI / 2.0;
    if (theta > PI)
        theta -= 2.0 * PI;

    end_left.x = goal.x + MINRADIUS * cos(theta);
    end_left.y = goal.y + MINRADIUS * sin(theta);
    end_left.radius = MINRADIUS;

    theta = goal.theta;
    theta -= PI / 2.0;
    if (theta < -PI)
        theta += 2.0 * PI;

    end_right.x = goal.x + MINRADIUS * cos(theta);
    end_right.y = goal.y + MINRADIUS * sin(theta);
    end_right.radius = MINRADIUS;
    DubinsPath CSC_path = get_CSCPath(start_left, start_right, end_left, end_right);
    DubinsPath CCC_path = get_CCCPath(start_left, start_right, end_left, end_right);
    if (CSC_path.length < CCC_path.length)
    {
        return CSC_path;
    }
    else
    {
        return CCC_path;
    }
}

double Dubins::get_arc_length(Point2D start, Point2D end, Circle center, bool is_right_turn)
{
    Point2D vec1, vec2;
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

DubinsPath Dubins::get_RSRPath(const vector<pair<Point2D, Point2D>> &tangent_points, const Circle &c1, const Circle &c2)
{

    if (tangent_points.size() == 0)
    {
        // std::cout << "No tangent points found" << std::endl;
        DubinsPath path;
        path.length = INFINITY;
        return path;
    }
    DubinsPath path;
    path.type = RSR;

    Point2D s(start.x, start.y);
    Point2D g(goal.x, goal.y);
    Point2D t1 = tangent_points.at(0).first;
    Point2D t2 = tangent_points.at(0).second;
    double arclength;

    // right turn
    arclength = get_arc_length(s, t1, c1, true);
    path.controls[0] = get_right_turn_control(arclength);
    path.length = arclength;

    // ahead
    arclength = get_euclidean_distance(t1.x, t1.y, t2.x, t2.y);
    path.controls[1] = get_no_turn_control(arclength);
    path.length += arclength;

    // right turn
    arclength = get_arc_length(t2, g, c2, true);
    path.controls[2] = get_right_turn_control(arclength);
    path.length += arclength;

    return path;
}

DubinsPath Dubins::get_RSLPath(const vector<pair<Point2D, Point2D>> &tangent_points, const Circle &c1, const Circle &c2)
{

    if (tangent_points.size() < 3)
    {
        // std::cout << "No tangent points found" << endl;
        DubinsPath path;
        path.length = INFINITY;
        return path;
    }
    DubinsPath path;
    path.type = RSL;

    Point2D s(start.x, start.y);
    Point2D g(goal.x, goal.y);
    Point2D t1 = tangent_points.at(2).first;
    Point2D t2 = tangent_points.at(2).second;
    double arclength;

    // right turn
    arclength = get_arc_length(s, t1, c1, true);
    path.controls[0] = get_right_turn_control(arclength);
    path.length = arclength;

    // ahead
    arclength = get_euclidean_distance(t1.x, t1.y, t2.x, t2.y);
    path.controls[1] = get_no_turn_control(arclength);
    path.length += arclength;

    // left turn
    arclength = get_arc_length(t2, g, c2, false);
    path.controls[2] = get_left_turn_control(arclength);
    path.length += arclength;

    return path;
}

DubinsPath Dubins::get_LSLPath(const vector<pair<Point2D, Point2D>> &tangent_points, const Circle &c1, const Circle &c2)
{
    if (tangent_points.size() < 2)
    {
        // std::cout << "No tangent points found" << endl;
        DubinsPath path;
        path.length = INFINITY;
        return path;
    }
    DubinsPath path;
    path.type = LSL;

    Point2D s(start.x, start.y);
    Point2D g(goal.x, goal.y);
    Point2D t1 = tangent_points.at(1).first;
    Point2D t2 = tangent_points.at(1).second;
    double arclength;

    // left turn
    arclength = get_arc_length(s, t1, c1, false);
    path.controls[0] = get_left_turn_control(arclength);
    path.length = arclength;

    // ahead
    arclength = get_euclidean_distance(t1.x, t1.y, t2.x, t2.y);
    path.controls[1] = get_no_turn_control(arclength);
    path.length += arclength;

    // left turn
    arclength = get_arc_length(t2, g, c2, false);
    path.controls[2] = get_left_turn_control(arclength);
    path.length += arclength;

    return path;
}

DubinsPath Dubins::get_LSRPath(const vector<pair<Point2D, Point2D>> &tangent_points, const Circle &c1, const Circle &c2)
{

    if (tangent_points.size() < 4)
    {
        // std::cout << "No tangent points found" << endl;
        DubinsPath path;
        path.length = INFINITY;
        return path;
    }

    DubinsPath path;
    path.type = LSR;

    Point2D s(start.x, start.y);
    Point2D g(goal.x, goal.y);
    Point2D t1 = tangent_points.at(3).first;
    Point2D t2 = tangent_points.at(3).second;
    double arclength;

    // left turn
    arclength = get_arc_length(s, t1, c1, false);
    path.controls[0] = get_left_turn_control(arclength);
    path.length = arclength;

    // ahead
    arclength = get_euclidean_distance(t1.x, t1.y, t2.x, t2.y);
    path.controls[1] = get_no_turn_control(arclength);
    path.length += arclength;

    // right turn
    arclength = get_arc_length(t2, g, c2, true);
    path.controls[2] = get_right_turn_control(arclength);
    path.length += arclength;

    return path;
}

DubinsPath Dubins::get_CSCPath(Circle start_left, Circle start_right, Circle end_left, Circle end_right)
{

    // print Circles
    // std::cout << "start_left: " << start_left.x << ", " << start_left.y << endl;
    // std::cout << "start_right: " << start_right.x << ", " << start_right.y << endl;
    // std::cout << "end_left: " << end_left.x << ", " << end_left.y << endl;
    // std::cout << "end_right: " << end_right.x << ", " << end_right.y << endl;

    // RSR
    vector<pair<Point2D, Point2D>> tangent_points = create_tangent(start_right, end_right);
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
    // std::cout << "RSR_length: " << RSR_length.length << endl;
    // std::cout << "LSL_length: " << LSL_length.length << endl;
    // std::cout << "RSL_length: " << RSL_length.length << endl;
    // std::cout << "LSR_length: " << LSR_length.length << endl;

    // get shortest path
    if (RSR_length.length < LSL_length.length && RSR_length.length < RSL_length.length && RSR_length.length < LSR_length.length)
    {
        return RSR_length;
    }
    else if (LSL_length.length < RSL_length.length && LSL_length.length < LSR_length.length)
    {
        return LSL_length;
    }
    else if (RSL_length.length < LSR_length.length)
    {
        return RSL_length;
    }
    else
    {
        return LSR_length;
    }
}

DubinsPath Dubins::get_RLRPath(const Circle &c1, const Circle &c2)
{
    double distance = get_euclidean_distance(c1.x, c1.y, c2.x, c2.y);
    double theta = acos(distance / (4 * MINRADIUS));
    if (distance > 4 * MINRADIUS)
    {
        // std::cout << "No tangent points found" << endl;
        DubinsPath path;
        path.length = INFINITY;
        return path;
    }
    DubinsPath path;
    path.type = RLR;

    Point2D vec1, vec2;
    vec1.x = c2.x - c1.x;
    vec1.y = c2.y - c1.y;
    double theta1 = atan2(vec1.y, vec1.x);
    // for an RLR path we want to subtract \theta from it to obtain a circle “to the right"
    theta = theta1 - theta;

    Circle c3;
    c3.x = c1.x + 2 * MINRADIUS * cos(theta);
    c3.y = c1.y + 2 * MINRADIUS * sin(theta);
    c3.radius = MINRADIUS;

    Point2D s(start.x, start.y);
    Point2D g(goal.x, goal.y);
    Point2D tangent_point1;
    tangent_point1.x = (c1.x + c3.x) / 2;
    tangent_point1.y = (c1.y + c3.y) / 2;

    Point2D tangent_point2;
    tangent_point2.x = (c2.x + c3.x) / 2;
    tangent_point2.y = (c2.y + c3.y) / 2;

    double arclength;

    // right turn
    arclength = get_arc_length(s, tangent_point1, c1, true);
    path.controls[0] = get_right_turn_control(arclength);
    path.length = arclength;

    // left turn
    arclength = get_arc_length(tangent_point1, tangent_point2, c3, false);
    path.controls[1] = get_left_turn_control(arclength);
    path.length += arclength;

    // right turn
    arclength = get_arc_length(tangent_point2, g, c2, true);
    path.controls[2] = get_right_turn_control(arclength);
    path.length += arclength;

    // PRINT RESULTS
    // std::cout << "arclength: " << arclength << ", timestamp: " << timestamp << endl;
    // std::cout << "arclength1: " << arclength1 << ", timestamp1: " << timestamp1 << endl;
    // std::cout << "arclength3: " << arclength3 << ", timestamp3: " << timestamp3 << endl;
    return path;
}

DubinsPath Dubins::get_LRLPath(const Circle &c1, const Circle &c2)
{
    double distance = get_euclidean_distance(c1.x, c1.y, c2.x, c2.y);
    double theta = acos(distance / (4 * MINRADIUS));
    if (distance > 4 * MINRADIUS)
    {
        // std::cout << "No tangent points found" << endl;
        DubinsPath path;
        path.length = INFINITY;
        return path;
    }
    DubinsPath path;
    path.type = LRL;
    Point2D vec1, vec2;
    vec1.x = c2.x - c1.x;
    vec1.y = c2.y - c1.y;
    double theta1 = atan2(vec1.y, vec1.x);
    // for an LRL path we want to add \theta from it to obtain a circle “to the right"
    theta = theta1 + theta;

    Circle c3;
    c3.x = c1.x + 2 * MINRADIUS * cos(theta);
    c3.y = c1.y + 2 * MINRADIUS * sin(theta);
    c3.radius = MINRADIUS;

    Point2D s(start.x, start.y);
    Point2D g(goal.x, goal.y);
    Point2D tangent_point1;
    tangent_point1.x = (c1.x + c3.x) / 2;
    tangent_point1.y = (c1.y + c3.y) / 2;

    Point2D tangent_point2;
    tangent_point2.x = (c2.x + c3.x) / 2;
    tangent_point2.y = (c2.y + c3.y) / 2;

    double arclength;

    // left turn
    arclength = get_arc_length(s, tangent_point1, c1, false);
    path.controls[0] = get_left_turn_control(arclength);
    path.length = arclength;

    // right turn
    arclength = get_arc_length(tangent_point1, tangent_point2, c3, true);
    path.controls[1] = get_right_turn_control(arclength);
    path.length += arclength;

    // left turn
    arclength = get_arc_length(tangent_point2, g, c2, false);
    path.controls[2] = get_left_turn_control(arclength);
    path.length += arclength;

    // PRINT RESULTS
    // std::cout << "arclength: " << arclength << ", timestamp: " << timestamp << endl;
    // std::cout << "arclength1: " << arclength1 << ", timestamp1: " << timestamp1 << endl;
    // std::cout << "arclength3: " << arclength3 << ", timestamp3: " << timestamp3 << endl;

    return path;
}

DubinsPath Dubins::get_CCCPath(Circle start_left, Circle start_right, Circle end_left, Circle end_right)
{

    // print Circles
    // std::cout << "start_left: " << start_left.x << ", " << start_left.y << endl;
    // std::cout << "start_right: " << start_right.x << ", " << start_right.y << endl;
    // std::cout << "end_left: " << end_left.x << ", " << end_left.y << endl;
    // std::cout << "end_right: " << end_right.x << ", " << end_right.y << endl;

    // RLR
    DubinsPath RLR_length = get_RLRPath(start_right, end_right);
    // std::cout << "RLR_length: " << RLR_length.length << endl;
    //  LRL
    DubinsPath LRL_length = get_LRLPath(start_left, end_left);
    // std::cout << "LRL_length: " << LRL_length.length << endl;

    // return the shortest path
    if (RLR_length.length < LRL_length.length)
    {
        return RLR_length;
    }
    else
    {
        return LRL_length;
    }
}

std::vector<PointDubins> Dubins::get_robot_trajectory(DubinsPath path)
{
    std::vector<PointDubins> points;

    if (path.length == INFINITY)
    {
        return points;
    }

    double x = start.x;
    double y = start.y;
    double theta = start.theta;

    for (DubinsControl control : path.controls)
    {
        if (control.timestamp == 0)
        {
            continue;
        }

        do
        {
            x += DELTA * cos(theta);
            y += DELTA * sin(theta);

            if (control.steering_angle != 0)
            {
                theta += DELTA / (WHEELBASE / sin(control.steering_angle));
                if (theta > PI)
                    theta -= 2.0 * PI;
                else if (theta < -PI)
                    theta += 2.0 * PI;
            }

            PointDubins p_tmp;
            p_tmp.x = x;
            p_tmp.y = y;
            p_tmp.theta = theta;
            points.push_back(p_tmp);

            control.timestamp--;
        } while (control.timestamp >= 0);
    }

    return points;
}

// int main(int argc, char *argv[])
// {
//     Point2D start;
//     start.x = 0;
//     start.y = 0;
//     double goal_x_ = std::stod(argv[1]);
//     double goal_y_ = std::stod(argv[2]);
//     Point2D goal;
//     goal.x = goal_x_;
//     goal.y = goal_y_;
//     double start.theta = 0;
//     double goal.theta = atan2(goal_y_, goal_x_);

//     Dubins dubins(start, goal, start.theta, goal.theta);
//     DubinsPath shortest_path = dubins.get_shortest_path();
//     cout << "------------------------" << endl;
//     std::cout << "Shortest path length: " << shortest_path.length << endl;
//     cout << "Shortest path type: " << shortest_path.type << endl;
//     cout << "Shortest path controls: " << endl;
//     for (int i = 0; i < 3; i++)
//     {
//         cout << "Control " << i << ": " << endl;
//         cout << "Timestamp: " << shortest_path.controls[i].timestamp << endl;
//         cout << "Steering angle: " << shortest_path.controls[i].steering_angle << endl;
//     }

//     if (shortest_path.length == INFINITY)
//     {
//         cout << "No path found" << endl;
//         return 0;
//     }
//     std::vector<PointDubins> points = dubins.get_robot_trajectory(shortest_path);
//     std::ofstream outfile("points.csv");
//     for (const auto &point : points)
//     {
//         outfile << point.x << "," << point.y << "," << point.theta << std::endl;
//     }
//     outfile.close();
//     system("python3 print.py");
//     return 0;
// }