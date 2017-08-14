#ifndef PATH_H
#define PATH_H

#include "geometry_msgs/Point.h"
#include <vector>
#include "ros/ros.h"
#include <string>
#include <math.h>

class path
{
private:
    path();

    void setup_node_list();
    bool setup_h();

    int find_closest_node(geometry_msgs::Point given_point);
    int find_pos(int id);

private:
    struct node{
        double x;
        double y;

        int id;

        std::vector<int> connected_nodes;

        double heuristic;

    };

private:
    geometry_msgs::Point _start_point;
    geometry_msgs::Point _end_point;

    std::vector<int> _path_order;
    std::vector<node> _node_list;

    ros::NodeHandle _n;

    int _start_id;
    int _end_id;

    int _start_pos;
    int _end_pos;


public:
    path(ros::NodeHandle nh);

    int update_current_point(geometry_msgs::Point start_point);
    int update_end_point(geometry_msgs::Point end_point);
};

#endif // PATH_H
