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
    int test2;
    std::string str;

    std::cout <<"before" << std::endl;
    n.getParam("/a_star/total_nodes", test2);

    std::cout << "TEST: " << test2 << std::endl;


    ros::spin();

}
