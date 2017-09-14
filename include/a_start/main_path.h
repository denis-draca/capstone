#ifndef MAIN_PATH_H
#define MAIN_PATH_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

class main_path
{
private://methods
    main_path();

    void a_star_path_callback(const geometry_msgs::PoseArrayConstPtr &msg);


private://members
    ros::NodeHandle _n;

    ros::Publisher _error_pub;
    ros::Publisher _path_pub;
    ros::Publisher _path_points_pub;

    ros::Subscriber _path_sub;

    geometry_msgs::PoseArray _a_star_path;

public:
    main_path(ros::NodeHandle &n);
    void find_steps();
};

#endif // MAIN_PATH_H
