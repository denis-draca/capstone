#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    ros::init( argc, argv, "a_star");

    std::cout << "************************************" << std::endl;

    ros::NodeHandle n;

    std::vector<int> test;

    n.getParam("/a_star/node_list", test);

    for (int i = 0; i < test.size(); i++)
    {
        std::cout <<" " << test.at(i) << std::endl;
    }


    ros::spin();

}
