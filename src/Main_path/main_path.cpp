#include "a_start/main_path.h"


main_path::main_path(ros::NodeHandle &n):
    _n(n)
{
    std::string error_name;
    std::string raw_path;

    _n.getParam("/main_path/errors/main_path", error_name);
    _n.getParam("/main_path/paths/a_star", raw_path);

    _error_pub = _n.advertise<std_msgs::String>(error_name.c_str(), 1);

    _path_sub = _n.subscribe(raw_path.c_str(), 1, &main_path::a_star_path_callback, this);



}

void main_path::a_star_path_callback(const geometry_msgs::PoseArrayConstPtr &msg)
{
    _a_star_path = *msg;
}
