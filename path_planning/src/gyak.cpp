#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std;

class RRTNode
{
public:
    double yaw;
    double x;
    double y;
    double cost;
    std::vector<double> path_x;
    std::vector<double> path_y;
    std::vector<double> path_yaw;
    int parent;

    RRTNode()
    {
        x = 0.0;
        y = 0.0;
        yaw = 0.0;
        cost = 0.0;
        parent = -1;
    }
    RRTNode(double x, double y, double yaw)
    {
        this->x = x;
        this->y = y;
        this->path_x = {};
        this->path_y = {};
        this->path_yaw = {};
        this->yaw = yaw;
        this->cost = 0.0;
        this->parent = -1;
    }

    // node << operator
};
typedef std::pair<double, double> Point2d;
#define MAX_ITER 25
#define GOAL_SAMPLE_RATE 5
class RRT
{
public:
    RRT(RRTNode start, RRTNode goal, std::vector<double> boundary)
    {
        this->start = RRTNode(start.x, start.y, start.yaw);
        this->end = RRTNode(goal.x, goal.y, goal.yaw);
        this->min_rand = boundary[0];
        this->max_rand = boundary[1];
        this->robot_radius = 0.5;
        node_list.push_back(start);
    }

    void print_node(int ind, RRTNode node)
    {
        std::cout << "ind: " << ind << " x: " << node.x << ", y: " << node.y << ", parent: " << node.parent << std::endl;

        // while (node.parent != -1)
        // {
        //     cout << node.parent << endl;
        //     node = node_list[node.parent];
        // }
        cout << endl;
    }

    std::vector<Point2d> planning()
    {
        std::vector<Point2d> path;

        for (int i = 0; i < MAX_ITER - 1; i++)
        {
            RRTNode rnd = get_random_node();
            int nearest_ind = get_closest_index(node_list, rnd);
            RRTNode new_node = get_path(&node_list[nearest_ind], rnd, nearest_ind);
            print_node(i, new_node);
            node_list.push_back(new_node);
        }

        RRTNode shortest = get_shortest_path();
        print_node(-1, shortest);
        if (shortest.x == 0 && shortest.y == 0)
        {
            cout << "No path found!" << endl;
            return path;
        }
        return get_final_path(shortest);
    }

private:
    double get_distance(double x, double y, double end_x, double end_y)
    {
        double distance = pow(end_x - end_x, 2) + pow(end_y - y, 2);
        return distance;
    }

    std::vector<Point2d> get_final_path(RRTNode node)
    {
        std::vector<Point2d> path;
        path.push_back(Point2d(end.x, end.y));
        path.push_back(Point2d(node.x, node.y));
        // print_node(-1, node);
        for (int i = node.path_x.size() - 1; i >= 0; i--)
        {
            double ix = node.path_x[i];
            double iy = node.path_y[i];
            path.push_back(Point2d(ix, iy));
            cout << "ix: " << ix << ", iy: " << iy << endl;
        }
        RRTNode parent_node = node_list[node.parent];
        int i = 0;
        while (parent_node.parent != -1)
        {
            // print_node(-1, parent_node);
            int input;
            path.push_back(Point2d(parent_node.x, parent_node.y));
            for (int i = parent_node.path_x.size() - 1; i >= 0; i--)
            {
                double ix = parent_node.path_x[i];
                double iy = parent_node.path_y[i];
                path.push_back(Point2d(ix, iy));
                cout << "ix: " << ix << ", iy: " << iy << endl;
            }
            int parent_index = parent_node.parent;
            parent_node.parent = -1;
            parent_node = node_list[parent_index];
        }
        path.push_back(Point2d(start.x, start.y));
        return path;
    }

    double calc_dist_to_goal(double x, double y, double end_x, double end_y)
    {
        double dx = x - end_x;
        double dy = y - end_y;
        return std::hypot(dx, dy);
    }

    RRTNode get_shortest_path()
    {
        cout << end.x << ", " << end.y << endl;
        std::vector<int> goal_indexes;
        for (int i = 0; i < node_list.size(); i++)
        {
            const RRTNode &node = node_list[i];
            double distance = get_distance(node.x, node.y, end.x, end.y);
            cout << "i: " << i << ", x: " << node.x << ", y: " << node.y << ", distance: " << distance << endl;
            if (distance <= 0.01)
            {
                goal_indexes.push_back(i);
            }
        }

        if (goal_indexes.empty())
        {
            return RRTNode();
        }

        double min_cost = node_list[goal_indexes[0]].cost;
        int best_goal_index = goal_indexes[0];
        for (int i : goal_indexes)
        {
            cout << "i: " << i << ", cost: " << node_list[i].cost << endl;
            if (node_list[i].cost < min_cost)
            {
                min_cost = node_list[i].cost;
                best_goal_index = i;
            }
        }
        cout << "best_goal_index: " << best_goal_index << endl;
        return node_list[best_goal_index];
    }

    RRTNode get_path(RRTNode *from_node, RRTNode to_node, int parent_index)
    {
        int sampling_rate = 10;
        std::vector<double> px, py, pyaw;
        double goal_x = to_node.x;
        double goal_y = to_node.y;
        double start_x = from_node->x;
        double start_y = from_node->y;
        double step_goal_x = (goal_x - start_x) / sampling_rate;
        double step_goal_y = (goal_y - start_y) / sampling_rate;

        for (int i = 0; i < sampling_rate; i++)
        {
            px.push_back(start_x + step_goal_x * i);
            py.push_back(start_y + step_goal_y * i);
            pyaw.push_back(0);
        }

        RRTNode new_node = RRTNode();
        new_node.path_x = px;
        new_node.path_y = py;
        new_node.path_yaw = pyaw;
        new_node.parent = parent_index;
        new_node.x = goal_x;
        new_node.y = goal_y;
        new_node.yaw = 0;
        new_node.cost = from_node->cost + get_distance(from_node->x, from_node->y, goal_x, goal_y);
        return new_node;
    }

    RRTNode get_random_node()
    {
        if (rand() % 100 > 10)
        {
            double rnd_x = ((double)rand() / RAND_MAX) * (max_rand - min_rand) + min_rand;
            double rnd_y = ((double)rand() / RAND_MAX) * (max_rand - min_rand) + min_rand;
            double rnd_yaw = ((double)rand() / RAND_MAX) * (2 * M_PI) - M_PI;
            return RRTNode(rnd_x, rnd_y, 0);
        }
        else
        {
            return RRTNode(end.x, end.y, end.yaw);
        }
    }

    int get_closest_index(std::vector<RRTNode> node_list, RRTNode rnd_node)
    {
        std::vector<double> dlist;
        for (const auto &node : node_list)
        {
            double distance = pow(node.x - rnd_node.x, 2) + pow(node.y - rnd_node.y, 2);
            dlist.push_back(distance);
        }

        // search for the smallest element in dlist and return its index
        std::vector<double>::iterator result = std::min_element(std::begin(dlist), std::end(dlist));
        return std::distance(std::begin(dlist), result);
    }

    RRTNode start;
    RRTNode end;
    std::vector<RRTNode> node_list;
    double robot_radius;
    double min_rand;
    double max_rand;
};

int initial_x_ = 0;
int initial_y_ = 0;
int goal_x_ = 2;
int goal_y_ = 2;

int main()
{
    RRTNode start(initial_x_, initial_y_, 0);
    RRTNode goal(goal_x_, goal_y_, 0);
    std::vector<double> boundary = {-3.1, 3.1};
    RRT rrt(start, goal, boundary);
    std::vector<Point2d> path = rrt.planning();
}