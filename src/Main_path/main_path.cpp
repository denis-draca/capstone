#include "a_start/main_path.h"


main_path::main_path(ros::NodeHandle &n):
    _n(n)
{
    std::string error_name;
    std::string raw_path;
    std::string calculated_paths;
    std::string path_points;
    std::string map_location;
    std::string shutdown_name;

    _n.getParam("/main_path/errors/main_path", error_name);
    _n.getParam("/main_path/paths/a_star", raw_path);
    _n.getParam("/main_path/paths/main_path", calculated_paths);
    _n.getParam("/main_path/paths/main_path_points", path_points);
    _n.getParam("/main_path/image", map_location);
    _n.getParam("main_path/shutdown", shutdown_name);

    _error_pub = _n.advertise<std_msgs::String>(error_name.c_str(), 1);
    _path_pub = _n.advertise<std_msgs::String>(calculated_paths.c_str(), 1);
    _path_points_pub = _n.advertise<geometry_msgs::PoseArray>(path_points.c_str(), 1);

    _path_sub = _n.subscribe(raw_path.c_str(), 1, &main_path::a_star_path_callback, this);
    _shutdown_sub = _n.subscribe(shutdown_name.c_str(), 1, &main_path::shutdown, this);

    _map = cv::imread(map_location.c_str(), CV_LOAD_IMAGE_GRAYSCALE);

    setup_landmarks();
}

void main_path::a_star_path_callback(const geometry_msgs::PoseArrayConstPtr &msg)
{
    _a_star_path = *msg;
    _path_pts.clear();

    for(int i = 0; i < _a_star_path.poses.size(); i++)
    {
        geometry_msgs::Pose pose = _a_star_path.poses.at(i);
        cv::Point2f pt;

        pt.x = pose.position.x;
        pt.y = pose.position.y;

        _path_pts.push_back(pt);
    }
}

void main_path::setup_landmarks()
{
    int count;

    _n.getParam("main_path/count", count);

    for(int i = 0; i < count; i++)
    {
        std::string index = std::to_string(i);
        std::string baseline = "main_path/landmark";

        baseline.append(index.c_str());

        std::vector<int> xy_coordinates;

        _n.getParam(baseline, xy_coordinates);

        landmark temp;
        temp.x = xy_coordinates.front();
        temp.y = xy_coordinates.back();

        temp.name = "landmark_";
        temp.name.append(std::to_string(i));

        _landmark_list.push_back(temp);


    }
}

void main_path::set_line_of_sights(geometry_msgs::Point start, geometry_msgs::Point goal)
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);


        cv::Point2f pt_landmark;
        cv::Point2f pt_start;
        cv::Point2f pt_goal;

        pt_landmark.x = temp.x;
        pt_landmark.y = temp.y;

        pt_start.x = start.x;
        pt_start.y = start.y;

        pt_goal.x = goal.x;
        pt_goal.y = goal.y;

        temp.start_line_of_sight = !check_intersection(pt_landmark, pt_start);
        temp.goal_line_of_sight = !check_intersection(pt_landmark, pt_goal);

        _landmark_list.at(i) = temp;
    }
}

void main_path::set_distances(geometry_msgs::Point start, geometry_msgs::Point goal)
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);

        cv::Point2f pt_landmark;
        cv::Point2f pt_start;
        cv::Point2f pt_goal;

        pt_landmark.x = temp.x;
        pt_landmark.y = temp.y;

        pt_start.x = start.x;
        pt_start.y = start.y;

        pt_goal.x = goal.x;
        pt_goal.y = goal.y;

        temp.dist_to_start = distance_between_two_points(pt_landmark, pt_start);
        temp.dist_to_goal = distance_between_two_points(pt_landmark, pt_goal);

        temp.already_passed = false;
        temp.closed = false;
        temp.open = false;

        _landmark_list.at(i) = temp;
    }
}

void main_path::publish_pts(std::vector<cv::Point2f> &list)
{
    geometry_msgs::PoseArray path;

    for(int i = 0; i < list.size(); i++)
    {
        cv::Point2f pt = list.at(i);

        geometry_msgs::Pose pose;

        pose.position.x = pt.x;
        pose.position.y = pt.y;

        path.poses.push_back(pose);
    }

    _path_points_pub.publish(path);
}

void main_path::shutdown(const std_msgs::BoolConstPtr &msg)
{
    std_msgs::String err_str;
    err_str.data = "WENT INTO SHUTDOWN";
    _error_pub.publish(err_str);


    ros::shutdown();
}

void main_path::set_closed(main_path::landmark &land)
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        if(land.name.compare(_landmark_list.at(i).name))
        {
            _landmark_list.at(i).closed = true;
        }
    }
}

void main_path::sort(std::vector<main_path::landmark> &list)
{
    for(int i = 0; i < list.size() - 1; i++)
    {
        for(int x = 0; x < list.size() - i - 1; x++)
        {
            if(list.at(x).cost < list.at(x + 1).cost)
            {
                landmark temp = list.at(x);
                list.at(x) = list.at(x + 1);
                list.at(x + 1) = temp;
            }
        }
    }
}

double main_path::distance_between_two_points(cv::Point2f &pt1, cv::Point2f &pt2)
{
    double dist;

    dist = sqrt(pow(pt1.x - pt2.x , 2) + pow(pt1.y - pt2.y , 2));

    return dist;
}

double main_path::distance_between_two_landmarks(main_path::landmark &land1, main_path::landmark &land2)
{
    cv::Point2f pt1;
    cv::Point2f pt2;


    pt1.x = land1.x;
    pt1.y = land1.y;

    pt2.x = land2.x;
    pt2.y = land2.y;


    return distance_between_two_points(pt2,pt1);
}

cv::Point2f main_path::landmark_position(std::string &landmark_name)
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);

        if(!temp.name.compare(landmark_name))
        {
            cv::Point2f pt;

            pt.x = temp.x;
            pt.y = temp.y;

            return pt;
        }
    }

    std_msgs::String str_err;

    str_err.data = "The landmark was not found within the list of landmarks. Chosen landmark must be wrong";

    _error_pub.publish(str_err);
}

bool main_path::check_intersection(cv::Point2f &pt1, cv::Point2f &pt2)
{
    cv::Mat img = _map.clone();

    cv::cvtColor(img,img, CV_GRAY2BGR);

    cv::Vec3b colour;

    colour[0] = 0;
    colour[1] = 0;
    colour[2] = 255;

//    cv::namedWindow("raycast", CV_WINDOW_NORMAL);

    double gradient = (pt1.y - pt2.y)/(pt2.x - pt2.x);

    for(int y = min(pt1.y,pt2.y); y <= max(pt1.y,pt2.y); y++)
    {
        for(int x = min(pt1.x, pt2.x); x <= max(pt1.x, pt2.x); x++)
        {
            double check = gradient*(x * pt1.x) + pt1.y;
            if(y <= check + 1 || y >= check + 1)
            {
//                img.at<cv::Vec3b>(y,x) = colour;

                if(_map.at<uchar>(y,x) < 250)
                {
                    return true;
                }
            }
        }
    }

//    cv::imshow("raycast", img);
//    cv::waitKey(3);

    return false;
}

bool main_path::can_i_see_a_landmark(cv::Point2f &pt1, std::string &closest_landmark)
{
    bool yes = false;
    double dist = 10000.0;
    int pos = 0;

    for(int i = 0; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);

        cv::Point2f pt_landmark;
        pt_landmark.x = temp.x;
        pt_landmark.y = temp.y;

        if(!check_intersection(pt1, pt_landmark))
        {
            if(temp.dist_to_goal < dist && !temp.already_passed)
            {
                yes = true;
                dist = temp.dist_to_goal;
                closest_landmark = temp.name;
                pos = i;
            }
        }

    }

    if(yes)
    {
        _landmark_list.at(pos).already_passed = true;
    }

    return yes;
}

bool main_path::landmark_can_see_goal(std::string &landmark_name)
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);

        if(!temp.name.compare(landmark_name))
        {
           return temp.goal_line_of_sight;
        }
    }

    std_msgs::String str_err;

    str_err.data = "The landmark was not found within the list of landmarks. Chosen landmark must be wrong";

    _error_pub.publish(str_err);
}

bool main_path::landmark_can_see_start(std::string &landmark_name)
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);

        if(!temp.name.compare(landmark_name))
        {
           return temp.start_line_of_sight;
        }
    }

    std_msgs::String str_err;

    str_err.data = "The landmark was not found within the list of landmarks. Chosen landmark must be wrong";

    _error_pub.publish(str_err);
}

bool main_path::closest_to_goal(std::string &closest_name)
{
    double smallest = _landmark_list.front().dist_to_goal;
    std::string name = _landmark_list.front().name;

    bool closest_can_see_goal = _landmark_list.front().goal_line_of_sight;

    for(int i = 1; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);

        if(temp.dist_to_goal < smallest && temp.goal_line_of_sight)
        {
            smallest = temp.dist_to_goal;
            name = temp.name;
            closest_can_see_goal = temp.goal_line_of_sight;
        }

    }

    closest_name = name;

    return closest_can_see_goal;
}

bool main_path::check_linked_landmarks(std::vector<std::string> &linked_list, std::string &closest_landmark)
{
    landmark last_one;
    bool linked = false;

    for(int i = 0; i < _landmark_list.size(); i++)
    {
        if(_landmark_list.at(i).name.compare(closest_landmark))
        {
            last_one = _landmark_list.at(i);
        }
    }


    std::vector<landmark> closed_list;
    std::vector<landmark> open_list;

    open_list.push_back(last_one);

    while(!open_list.empty())
    {
        sort(open_list);
        closed_list.push_back(open_list.back());
        open_list.pop_back();

        landmark current = closed_list.back();
        set_closed(current);

        if(current.start_line_of_sight)
        {
            linked = true;
            break;
        }

        for(int i = 0; i < _landmark_list.size(); i++)
        {
            if(_landmark_list.at(i).closed)
                continue;

            if(line_of_sight(current, _landmark_list.at(i)) && !_landmark_list.at(i).open)
            {
                _landmark_list.at(i).cost = distance_between_two_landmarks(current, _landmark_list.at(i));
                _landmark_list.at(i).open = true;
                open_list.push_back(_landmark_list.at(i));
            }
        }
    }

    if(linked)
    {
        while(!closed_list.empty())
        {
            std::string str;

            landmark temp = closed_list.back();
            closed_list.pop_back();

            str = temp.name;

            if(temp.goal_line_of_sight)
            {
                linked_list.push_back(str);
                break;
            }

            linked_list.push_back(str);
        }

    }




    return linked;
}

bool main_path::line_of_sight(main_path::landmark &land1, main_path::landmark &land2)
{
    cv::Point2f pt1;
    cv::Point2f pt2;


    pt1.x = land1.x;
    pt1.y = land1.y;

    pt2.x = land2.x;
    pt2.y = land2.y;


    if(check_intersection(pt1,pt2))
        return false;

    return true;
}

double main_path::max(double x, double y)
{
    if(x > y)
    {
        return x;
    }

    return y;
}

double main_path::min(double x, double y)
{
    if(x < y)
    {
        return x;
    }

    return y;
}

void main_path::find_steps()
{
    if(!ros::ok())
    {
        return;
    }
    if(_a_star_path.poses.empty())
    {
        std_msgs::String err_str;
        err_str.data = "NO PATH TO USE";
        _error_pub.publish(err_str);
        return;
    }

    geometry_msgs::Point start;
    geometry_msgs::Point goal;

    start = _a_star_path.poses.front().position;
    goal =_a_star_path.poses.back().position;

    set_line_of_sights(start, goal);
    set_distances(start, goal);


    bool found = false;
    int upto = 0;

    std::string directions = "Path Directions: ";

    std::vector<cv::Point2f> point_list;

    point_list.push_back(_path_pts.front());


    //----------------------------------------------------------------------------
    //                              New Path Here
    //----------------------------------------------------------------------------

    bool goal_found = false;

    std::string closest_landmark;
    bool landmark_can_see = closest_to_goal(closest_landmark);
    std::vector<std::string> list;

    if(!check_intersection(_path_pts.front(), _path_pts.back()))
    {
        directions.append("\nYou can see the goal from here");
        goal_found = true;
    }
    else if(landmark_can_see)
    {
        if(landmark_can_see_start(closest_landmark))
        {
            directions.append("\nGo past ");
            directions.append(closest_landmark);
            directions.append(" You can see the goal from there");

            point_list.push_back(landmark_position(closest_landmark));
            goal_found = true;
        }
        else if(check_linked_landmarks(list, closest_landmark))
        {
            for(int i = 0; i < list.size(); i++)
            {
                directions.append("\nGo past ");
                directions.append(list.at(i));

                point_list.push_back(landmark_position(list.at(i)));
            }

            directions.append(" You can see the goal from there");
            goal_found = true;
        }
    }

    while(!goal_found)
    {

    }



    //----------------------------------------------------------------------------
    //                              End New Path Here
    //----------------------------------------------------------------------------



    point_list.push_back(_path_pts.back());
    std_msgs::String msg;

    msg.data = directions.c_str();

    _path_pub.publish(msg);

    publish_pts(point_list);

}
