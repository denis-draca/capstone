#ifndef MAIN_PATH_H
#define MAIN_PATH_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <string>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "opencv2/opencv.hpp"

class main_path
{
private://methods
    main_path();

    //Returns VOID
    void a_star_path_callback(const geometry_msgs::PoseArrayConstPtr &msg);
    void setup_landmarks();
    void set_line_of_sights(geometry_msgs::Point start, geometry_msgs::Point goal);
    void set_distances(geometry_msgs::Point start, geometry_msgs::Point goal);
    void publish_pts(std::vector<cv::Point2f> &list);
    void shutdown(const std_msgs::BoolConstPtr &msg);

    //Returns BOOL
    bool check_intersection(cv::Point2f &pt1, cv::Point2f &pt2);
    bool can_i_see_a_landmark(cv::Point2f &pt1, std::string &closest_landmark);
    bool landmark_can_see_goal(std::string &landmark_name);

    //Returns Double
    double max(double x, double y);
    double min(double x, double y);
    double distance_between_two_points(cv::Point2f &pt1, cv::Point2f &pt2);

    //pt returns

    cv::Point2f landmark_position(std::string &landmark_name);


private://members

    struct landmark{
        double x;
        double y;

        std::string name;

        bool goal_line_of_sight;
        bool start_line_of_sight;
        bool already_passed;

        double dist_to_start;
        double dist_to_goal;



    };

    //ROS STUFF
    ros::NodeHandle _n;

    ros::Publisher _error_pub;
    ros::Publisher _path_pub;
    ros::Publisher _path_points_pub;

    ros::Subscriber _path_sub;
    ros::Subscriber _shutdown_sub;

    geometry_msgs::PoseArray _a_star_path;

    //c++ standard libraries
    std::vector<landmark> _landmark_list;
    std::vector<cv::Point2f> _path_pts;

    //OPENCV STUFF
    cv::Mat _map;

public:
    main_path(ros::NodeHandle &n);
    void find_steps();
};

#endif // MAIN_PATH_H
