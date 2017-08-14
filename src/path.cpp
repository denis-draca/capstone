#include "a_start/path.h"

path::path(ros::NodeHandle nh):
    _n(nh)
{
    setup_node_list();

    _start_id = -1;
    _end_id = -1;

}

void path::setup_node_list()
{
    std::vector<int> node_id;

    _n.getParam("/a_star/node_list", node_id);

    std::string read_node = "/a_star/node_";

    for(int i = 0; i < node_id.size(); i++)
    {
        node temp;

        std::string read_temp = read_node;
        read_temp.append(std::to_string(node_id.at(i)));

        std::string x_str = read_temp;
        std::string y_str = read_temp;
        std::string connected_nodes = read_temp;

        x_str.append("/x");
        y_str.append("/y");
        connected_nodes.append("/connected_nodes");

        double x;
        double y;

        std::vector<int> temp_list;

        _n.getParam(x_str.c_str(), x);
        _n.getParam(y_str.c_str(), y);
        _n.getParam(connected_nodes.c_str(), temp_list);

        temp.x = x;
        temp.y = y;
        temp.connected_nodes = temp_list;
        temp.id = node_id.at(i);

        _node_list.push_back(temp);

    }
}

bool path::setup_h()
{
    if(_end_id < 0 || _start_id < 0)
    {
        return false;
    }

    for(int i = 0; i < _node_list.size(); i++)
    {
        if(i = _end_pos)
        {
            continue;
        }

        double dist = sqrt(pow(_node_list.at(i).x - _node_list.at(_end_pos).x, 2) + pow(_node_list.at(i).y - _node_list.at(_end_pos).y, 2));

        _node_list.at(i).heuristic = dist;
    }

    return true;
}

int path::find_closest_node(geometry_msgs::Point given_point)
{
    if(_node_list.empty())
    {
        return -1;
    }

    double min = sqrt(pow(given_point.x - _node_list.front().x, 2) + pow(given_point.y - _node_list.front().y, 2));
    int found_id = 0;

    for (int i = 1; i < _node_list.size(); i++)
    {
        double temp = sqrt(pow(given_point.x - _node_list.at(i).x, 2) + pow(given_point.y - _node_list.at(i).y, 2));

        if (temp < min)
        {
            min = temp;
            found_id = _node_list.at(1).id;
        }
    }

    return found_id;
}

int path::find_pos(int id)
{
    if(id < 0)
    {
        return -1;
    }

    for(int i = 0; i < _node_list.size(); i++)
    {
        if(_node_list.at(i).id == id)
        {
            return i;
        }
    }

    return -1;
}

int path::update_current_point(geometry_msgs::Point start_point)
{
    _start_point = start_point;

    _start_id = find_closest_node(_start_point);
    _start_pos = find_pos(_start_id);

    return _start_id;

}

int path::update_end_point(geometry_msgs::Point end_point)
{
    _end_point = end_point;

    _end_id = find_closest_node(_end_point);
    _end_pos = find_pos(_end_id);

    return _end_id;
}
