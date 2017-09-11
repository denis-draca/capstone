#include "a_start/path.h"

path::path(ros::NodeHandle nh):
    _n(nh)
{
    ROS_INFO("STARTING NODE SETUP");
    setup_node_list();
    ROS_INFO("NODE SETUP DONE");

    _start_id = -1;
    _end_id = -1;

    _path_pub = _n.advertise<geometry_msgs::PoseArray>("/capstone/raw/path",1);

    _start_sub = _n.subscribe("/capstone/path/start", 1, &path::start_point_callback, this);
    _end_sub = _n.subscribe("/capstone/path/end", 1, &path::end_point_callback, this);

    img_open = cv::imread("/home/denis/catkin_ws/src/a_start/data/map4.png", CV_LOAD_IMAGE_COLOR);
    img_for_path = cv::imread("/home/denis/catkin_ws/src/a_start/data/map4.png", CV_LOAD_IMAGE_GRAYSCALE);


    _given_start_point.x = 1.1;
    _given_start_point.y = 3.0;

    _given_end_point.x = 40;
    _given_end_point.y = 10;



}

int path::find_path()
{

    std::vector<std::vector<bool>> explored;

    for(int y = 0; y < img_for_path.rows; y++)
    {
        std::vector<bool> temp;
        for(int x = 0; x < img_for_path.cols; x++)
        {
            temp.push_back(false);
        }

        explored.push_back(temp);
    }

    ROS_INFO("Resetting nodes");
    reset_nodes();

    cv::Point2f pt_s;
    cv::Point2f pt_e;


    ROS_INFO("UPRATE POINTS");
    update_current_point(_given_start_point);
    update_end_point(_given_end_point);


    ROS_INFO("UPRATE POINTS DONE");

    pt_s.x = _given_start_point.x;
    pt_s.y = _given_start_point.y;

    std::cout << _end_id << std::endl;

    pt_e.x = _node_list.at(_end_pos).x;
    pt_e.y = _node_list.at(_end_pos).y;


    std::vector<node> open_list;
    std::vector<node> closed_list;
    std::vector<node> total_open_list;

    if(_given_start_point.x == _given_end_point.x && _given_start_point.y == _given_end_point.y)
    {
        ROS_ERROR("START POINT THE SAME AS END POINT");
        return -1;
    }

    bool _found = false;

    closed_list.push_back(_node_list.at(_start_pos));

    std::cout << "start id: " << _node_list.at(_start_pos).id << " END ID: " << _end_id << std::endl;

    while(!_found)
    {
        node current_node = closed_list.back();
        geometry_msgs::Point current_node_pt;

        current_node_pt.x = current_node.x;
        current_node_pt.y = current_node.y;

        double dist = sqrt(pow((pt_e.x - current_node.x),2) + pow((pt_e.y - current_node.y),2));

        if(current_node.id == _end_id || dist < 3)
        {
            ROS_INFO("REACHED END NODE");
            _found = true;
            break;
        }

        for(int i = 0; i < current_node.connected_nodes.size(); i++)
        {
            try
            {
                int node_pos = find_pos(current_node.connected_nodes.at(i));

                if(node_pos < 0)
                {
                    std::cout << "Weird Access: " << node_pos << " Searched ID: " << current_node.connected_nodes.at(i) << " " << current_node.id<<std::endl;
                    continue;
                }

                node temp = _node_list.at(node_pos);

                if(temp.closed || temp.open)
                {
                    continue;
                }

                _node_list.at(node_pos).open = true;

                node start = _node_list.at(_start_pos);

                temp.heuristic = sqrt(pow(start.x - temp.x,2) + pow(start.y - temp.y,2));

                temp.heuristic += sqrt(pow(temp.x - _node_list.at(_end_pos).x,2) + pow(temp.y - _node_list.at(_end_pos).y,2));

//                temp.heuristic = 1;
//                temp.heuristic += std::abs(temp.x - _node_list.at(_end_pos).x) + std::abs(temp.y - _node_list.at(_end_pos).y);


                temp.pos = node_pos;

                open_list.push_back(temp);
                total_open_list.push_back(temp);
            }
            catch(std::exception e)
            {
                std::cout << "WHAT: " << e.what() << std::endl;
            }

        }

        if(!sort_list(open_list))
        {
            ROS_ERROR("OPEN LIST IS EMPTY, NO PATH");
            return -2;
        }

        closed_list.push_back(open_list.back());

        open_list.pop_back();
        open_list.clear();

        display_list(open_list, img_open, "open list", 127,127,127, false);
        display_list(closed_list, img_open, "closed list", 0,255,0, false);
        display_list(total_open_list, img_open, "unchanged open list", 0,0,255, false);
//        print_h(total_open_list);

    }

    ROS_INFO("FINISHED SEARCH");

    sort_list(closed_list);

    publish_path(closed_list);

}

void path::setup_node_list()
{
//    std::vector<int> node_id;

//    _n.getParam("/a_star/node_list", node_id);

//    std::string read_node = "/a_star/node_";

//    for(int i = 0; i < node_id.size(); i++)
//    {
//        node temp;

//        std::string read_temp = read_node;
//        read_temp.append(std::to_string(node_id.at(i)));

//        std::string x_str = read_temp;
//        std::string y_str = read_temp;
//        std::string connected_nodes = read_temp;

//        x_str.append("/x");
//        y_str.append("/y");
//        connected_nodes.append("/connected_nodes");

//        double x;
//        double y;

//        std::vector<int> temp_list;

//        _n.getParam(x_str.c_str(), x);
//        _n.getParam(y_str.c_str(), y);
//        _n.getParam(connected_nodes.c_str(), temp_list);

//        temp.x = x - img_open.cols/2;
//        temp.y = img_open.rows/2 - y;
//        temp.connected_nodes = temp_list;
//        temp.id = node_id.at(i);

//        temp.open = false;
//        temp.closed = false;

//        _node_list.push_back(temp);

//    }


    cv::Mat img = cv::imread("/home/denis/catkin_ws/src/a_start/data/map4.png", CV_LOAD_IMAGE_GRAYSCALE);
    for(int y = 0; y < img.rows; y++)
    {
        for(int x = 0; x < img.cols; x++)
        {
            if(img.at<uchar>(y,x) < 250)
            {
                continue;
            }

            node temp;

            temp.x = x;
            temp.y = y;

            temp.id = y * img.cols + x;

            temp.open = false;
            temp.closed = false;

            for(int y_temp = y - 1; y_temp < y + 2; y_temp++)
            {
                for(int x_temp = x - 1; x_temp < x + 2; x_temp++)
                {
                    if(y_temp < 0 || y_temp > img.rows || x_temp < 0 || x_temp > img.cols)
                    {
                        continue;
                    }

                    if(img.at<uchar>(y_temp,x_temp) < 250)
                    {
                        continue;
                    }

                    if(y_temp == y && x_temp == x)
                    {
                        continue;
                    }


                    temp.connected_nodes.push_back(y_temp * img.cols + x_temp);

                }

            }


            _node_list.push_back(temp);
        }
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

    double min = sqrt(pow((given_point.x) - _node_list.front().x, 2) + pow((given_point.y) - _node_list.front().y, 2));
    int found_id = 0;

    for (int i = 1; i < _node_list.size(); i++)
    {
        double temp = sqrt(pow((given_point.x) - _node_list.at(i).x, 2) + pow((given_point.y) - _node_list.at(i).y, 2));

        if (temp < min)
        {
            min = temp;
            found_id = _node_list.at(i).id;
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

    return -2;
}

bool path::sort_list(std::vector<node> &list)
{
    if(list.empty())
    {
        ROS_ERROR("LIST IS EMPTY");
        return false;
    }

    for(int i = 0; i < list.size() - 1; i++)
    {
        for(int x = 0; x < list.size() - i - 1; x++)
        {
            if(list.at(x).heuristic < list.at(x + 1).heuristic)
            {
                node temp = list.at(x);
                list.at(x) = list.at(x + 1);
                list.at(x + 1) = list.at(x);
            }
        }
    }

    return true;
}

int path::update_current_point(geometry_msgs::Point start_point)
{
    _start_point = start_point;
    std::cout << "START X: " << _start_point.x << " START Y: " << _start_point.y << std::endl;

    _start_id = find_closest_node(_start_point);
    std::cout << "START ID " << _start_id << std::endl;
    _start_pos = find_pos(_start_id);
    std::cout << "START POS " << _start_pos << std::endl;

    return _start_id;

}

int path::update_end_point(geometry_msgs::Point end_point)
{
    _end_point = end_point;

    std::cout << "END X: " << _end_point.x << " END Y: " << _end_point.y << std::endl;
    _end_id = find_closest_node(_end_point);
    std::cout << "END ID" << _end_id << std::endl;
    _end_pos = find_pos(_end_id);
    std::cout << "END pos" << _end_pos << std::endl;

    return _end_id;
}

bool path::display_list(std::vector<path::node> &list, cv::Mat &img, char name[], int b, int g, int r, bool lines = false)
{
    cv::Mat made_img;


 // MAKE STUFF HERE
    if(lines)
    {
        made_img = img;
        node goal = _node_list.at(_end_pos);
        cv::Point2f pt_end;

        pt_end.x = goal.x;
        pt_end.y = goal.y;

        cv::Point2f pt_front;
        node front = list.front();

        pt_front.x = front.x;
        pt_front.y = front.y;

        cv::line(made_img, pt_end, pt_front, cv::Scalar(b,g,r), 1);

        for(int i = 0; i < list.size() - 1; i++)
        {
            node temp = list.at(i);
            node temp2 = list.at(i + 1);

            cv::Point2f pt;
            cv::Point2f pt2;

            pt.x = temp.x + img.cols/2;
            pt.y = img.rows/2 - temp.y;

            pt2.x = temp2.x + img.cols/2;
            pt2.y = img.rows/2 - temp2.y;

            cv::line(made_img, pt, pt2, cv::Scalar(b,g,r), 1);

        }


    }
    else
    {
        cv::namedWindow(name, CV_WINDOW_NORMAL);
        made_img = img.clone();

        cv::Point2f pt;

        pt.x = _start_point.x;
        pt.y = _start_point.y;

        cv::circle(made_img, pt, 1, cv::Scalar(0,255,0));

        pt.x = _end_point.x ;
        pt.y = _end_point.y;

        cv::circle(made_img, pt, 1, cv::Scalar(0,0,255));

        for(int i = 0; i < list.size(); i++)
        {
            node temp = list.at(i);

            cv::Point2f pt;
            pt.x = temp.x;
            pt.y = temp.y;

            if(pt.x > made_img.cols || pt.x < 0 || pt.y > made_img.rows || pt.y < 0)
            {
                ROS_ERROR("ACCESSING BEYOND IMAGE");
                return false;
            }

            cv::Vec3b color;
            color[0] = b;
            color[1] = g;
            color[2] = r;
            made_img.at<cv::Vec3b>(pt) = color;

        }
        cv::imshow(name, made_img);
        cv::waitKey(3);
    }


}

void path::reset_nodes()
{
    for(int i = 0; i < _node_list.size(); i++)
    {
        _node_list.at(i).open = false;
        _node_list.at(i).closed = false;
    }
}

void path::publish_path(std::vector<path::node> &list)
{

    geometry_msgs::PoseArray path;


    while(!list.empty())
    {
        node temp = list.back();

        geometry_msgs::Point pt;
        geometry_msgs::Pose pose;


        pt.x = temp.x;
        pt.y = temp.y;

        pose.position = pt;

        if (temp.id == _start_id)
        {
            path.poses.push_back(pose);
            break;
        }

        path.poses.push_back(pose);

        list.pop_back();
    }

    ROS_INFO("PUBLISHED");
    _path_pub.publish(path);
}

void path::start_point_callback(const geometry_msgs::PointConstPtr &start_msg)
{
    _given_start_point = *start_msg;
}

void path::end_point_callback(const geometry_msgs::PointConstPtr &end_msg)
{
    _given_end_point = *end_msg;
}

void path::print_h(std::vector<node> &node_list)
{
    cv::namedWindow("H", cv::WINDOW_NORMAL);
    cv::Mat img = img_open.clone();
    cv::cvtColor(img,img, CV_BGR2GRAY);

    for(int i = 0; i < node_list.size(); i++)
    {
        node temp = node_list.at(i);

        int x = temp.x + img_open.cols/2;
        int y = img_open.rows/2 - temp.y;

        img.at<uchar>(y,x) = 0;

        img.at<uchar>(y,x) = temp.heuristic;
    }

    cv::imshow("H", img);
    cv::waitKey(3);
}

double path::distance_between_nodes(path::node &node1, path::node &node2)
{
    double dist = 0.0;

    dist = sqrt(pow((node2.x - node1.x),2) + pow((node2.y - node1.y),2));

    return dist;
}

