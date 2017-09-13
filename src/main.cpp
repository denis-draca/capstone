#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <string>
#include <vector>
#include "a_start/path.h"
#include "a_start/nodegen.h"

int main(int argc, char** argv)
{

    ros::init( argc, argv, "a_star");

    std::cout << "************************************" << std::endl;


    ros::NodeHandle n;

    path _path(n);

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        _path.find_path();

        ros::spinOnce();
        loop_rate.sleep();
    }


//    ros::spin();

}
