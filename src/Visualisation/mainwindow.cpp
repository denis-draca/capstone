#include "a_start/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(ros::NodeHandle &n, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow), _n(n)
{
    ui->setupUi(this);

    multiplier = 8;

    std::string error_out_a_star;
    std::string error_out_main;
    std::string raw_path;
    std::string directions;
    std::string direction_pts;
    std::string shutdown_name;

    _n.getParam("/visualiser/errors/a_star", error_out_a_star);
    _n.getParam("/visualiser/errors/main_path", error_out_main);
    _n.getParam("/visualiser/paths/a_star", raw_path);
    _n.getParam("/visualiser/paths/main_path", directions);
    _n.getParam("/visualiser/paths/main_path_points", direction_pts);
    _n.getParam("/visualiser/shutdown", shutdown_name);

    _path_sub = _n.subscribe(raw_path.c_str(), 1, &MainWindow::path_callback,this);
    _error_a_start_sub = _n.subscribe(error_out_a_star.c_str(),1, &MainWindow::a_start_error_callback, this);
    _error_main_sub = _n.subscribe(error_out_main.c_str(), 1, &MainWindow::main_path_error_callback, this);
    _path_directions_sub = _n.subscribe(directions.c_str(), 1 , &MainWindow::directions_callback, this);
    _path_points_sub = _n.subscribe(direction_pts.c_str(), 1, &MainWindow::direction_pts_callback, this);

    _start_point_pub = _n.advertise<geometry_msgs::Point>("/capstone/path/start", 1);
    _end_point_pub = _n.advertise<geometry_msgs::Point>("/capstone/path/end", 1);
    _shutdown_pub = _n.advertise<std_msgs::Bool>(shutdown_name.c_str(), 1);


    QString fileName = "/home/denis/catkin_ws/src/a_start/data/map4.png"; /*QFileDialog::getOpenFileName(this,"Open Image File",QDir::currentPath())*/

    if(!fileName.isEmpty())
    {
        QImage image(fileName);

        if(image.isNull())
        {
            std::cout << "ERROR" << std::endl;
            QMessageBox::information(this,"Image Viewer","Error Displaying image");
            return;
        }

        path_img = cv::imread("/home/denis/catkin_ws/src/a_start/data/map4.png", CV_LOAD_IMAGE_COLOR);

        cv::Mat temp = path_img.clone();

        cv::Size size(path_img.cols * multiplier, path_img.rows * multiplier);
        cv::resize(path_img, temp, size);

        ui->img_label->setPixmap(QPixmap::fromImage(Mat2QImage(temp)));

        ui->in_start_x->setValidator(new QIntValidator(0, path_img.cols, this));
        ui->in_start_y->setValidator(new QIntValidator(0, path_img.rows, this));
        ui->in_end_x->setValidator(new QIntValidator(0, path_img.cols, this));
        ui->in_end_y->setValidator(new QIntValidator(0, path_img.rows, this));
    }


    setup_landmarks();
    display_landmarks();

    _timer = new QTimer(this);
     connect(_timer, SIGNAL(timeout()), this, SLOT(check_callbacks()));
    _timer->start(100);

    _error_show = true;
}

cv::Mat MainWindow::QImage2Mat(const QImage &src)
{
//    cv::Mat tmp(src.height(), src.width(), CV_8UC, (uchar*)src.bits(), src.bytesPerLine());
//    cv::Mat result;
//    cvtColor(tmp, result, CV_BGR2RGB);

    //    return result;
}

void MainWindow::path_callback(const geometry_msgs::PoseArrayConstPtr &msg)
{

    geometry_msgs::PoseArray derivedMsg = *msg;

    cv::Mat temp;

    cv::Mat display_path = path_to_img(derivedMsg);

    cv::Size size(display_path.cols * multiplier, display_path.rows * multiplier);
    cv::resize(display_path, temp, size);

    ui->img_label->setPixmap(QPixmap::fromImage(Mat2QImage(temp)));
}

cv::Mat MainWindow::path_to_img(geometry_msgs::PoseArray &path)
{
    cv::Mat img = path_img.clone();

    for(int i = 1; i < path.poses.size(); i++)
    {
        cv::Point2f pt1;
        cv::Point2f pt2;

        geometry_msgs::Pose pose1 = path.poses.at(i-1);
        geometry_msgs::Pose pose2 = path.poses.at(i);

        pt1.x = pose1.position.x;
        pt1.y = pose1.position.y;

        pt2.x = pose2.position.x;
        pt2.y = pose2.position.y;

        cv::line(img, pt1, pt2, cv::Scalar(255,0,0));

    }

    cv::Point2f pt1;
    cv::Point2f pt2;

    pt1.x = _start_x;
    pt1.y = _start_y;

    pt2.x = _end_x;
    pt2.y = _end_y;

    cv::circle(img, pt1, 1, cv::Scalar(0,255,0));
    cv::circle(img, pt2, 1, cv::Scalar(0,0,255));


    return img;
}

void MainWindow::update_inputs()
{
    std::string start_x = ui->in_start_x->text().toUtf8().constData();
    std::string start_y = ui->in_start_y->text().toUtf8().constData();
    std::string end_x = ui->in_end_x->text().toUtf8().constData();
    std::string end_y = ui->in_end_y->text().toUtf8().constData();


    if(start_x.empty())
        _start_x = -1;
    else
        _start_x = std::stoi(start_x);

    if(start_y.empty())
        _start_y = -1;
    else
        _start_y = std::stoi(start_y);

    if(end_x.empty())
        _end_x = -1;
    else
        _end_x = std::stoi(end_x);

    if(end_y.empty())
        _end_y = -1;
    else
        _end_y = std::stoi(end_y);

    if(start_x.empty() && start_y.empty() && end_x.empty() && end_y.empty())
    {
        return;
    }

}

void MainWindow::setup_landmarks()
{
    int count;

    _n.getParam("visualiser/count", count);

    for(int i = 0; i < count; i++)
    {
        std::string index = std::to_string(i);
        std::string baseline = "visualiser/landmark";

        baseline.append(index.c_str());

        std::vector<int> xy_coordinates;

        _n.getParam(baseline, xy_coordinates);

        std::pair<int,int> temp;
        temp.first = xy_coordinates.front();
        temp.second = xy_coordinates.back();

        _landmarks.push_back(temp);
    }


}

void MainWindow::display_landmarks()
{
    cv::Mat landmark_disp = path_img.clone();

    for(int i = 0; i < _landmarks.size(); i++)
    {
        std::pair<int,int> temp = _landmarks.at(i);

        cv::Point2f pt;

        pt.x = temp.first;
        pt.y = temp.second;

//        ROS_ERROR("X %d Y %d", temp.first, temp.second);
        cv::circle(landmark_disp, pt, 1, cv::Scalar(50, 100, 200));
    }

    cv::Mat temp;

    cv::Size size(landmark_disp.cols * multiplier, landmark_disp.rows * multiplier);
    cv::resize(landmark_disp, temp, size);

    ui->img_landmarks->setPixmap(QPixmap::fromImage(Mat2QImage(temp)));

}

void MainWindow::a_start_error_callback(const std_msgs::String &msg)
{
    _a_start_error.append("\n");
    _a_start_error.append(msg.data);

    if(_error_show)
    ui->out_ASTART_error->setText(_a_start_error.c_str());
}

void MainWindow::main_path_error_callback(const std_msgs::String &msg)
{
    _main_path_error.append("\n");
    _main_path_error.append(msg.data);

    if(_error_show)
    ui->out_MAIN_error->setText(_main_path_error.c_str());
}

void MainWindow::directions_callback(const std_msgs::String &msg)
{
    std::string str = msg.data;
    ui->out_directions->setText(str.c_str());
}

void MainWindow::direction_pts_callback(const geometry_msgs::PoseArrayConstPtr &msg)
{
    geometry_msgs::PoseArray pt_list = *msg;

    if(pt_list.poses.empty())
    {
        _main_path_error.append("\n");
        _main_path_error.append("empty path transmitted");
        ui->out_MAIN_error->setText(_main_path_error.c_str());
        return;
    }

    cv::Mat img = path_img.clone();

    for(int i = 0; i < pt_list.poses.size(); i++)
    {
        cv::Point2f pt;

        pt.x = pt_list.poses.at(i).position.x;
        pt.y = pt_list.poses.at(i).position.y;

        cv::circle(img, pt, 1, cv::Scalar(200,100,100));
    }

    cv::Mat temp;

    cv::Size size(img.cols * multiplier, img.rows * multiplier);
    cv::resize(img, temp, size);

    ui->img_directions->setPixmap(QPixmap::fromImage(Mat2QImage(temp)));

}

MainWindow::~MainWindow()
{
    delete ui;
}

QImage MainWindow::Mat2QImage(cv::Mat const& src)
{
    cv::Mat temp;
    cv::cvtColor(src, temp, CV_BGR2RGB);
    QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    dest.bits();

    return dest;
}

void MainWindow::on__bu_find_path_clicked()
{
    update_inputs();

    geometry_msgs::Point start_pt;
    geometry_msgs::Point end_pt;

    if(!_start_x)
        start_pt.x = 0;
    else
        start_pt.x = _start_x;

    if(!_start_y)
        start_pt.y = 0;
    else
        start_pt.y = _start_y;

    if(!_end_x)
        end_pt.x = 0;
    else
       end_pt.x = _end_x;

    if(!_end_y)
        end_pt.y = 0;
    else
        end_pt.y = _end_y;

    cv::Mat temp;

    cv::Mat display_path = path_img.clone();

    cv::Point2f pt1;
    cv::Point2f pt2;

    pt1.x = _start_x;
    pt1.y = _start_y;

    pt2.x = _end_x;
    pt2.y = _end_y;

    cv::circle(display_path, pt1, 1, cv::Scalar(0,255,0));
    cv::circle(display_path, pt2, 1, cv::Scalar(0,0,255));

    cv::Size size(display_path.cols * multiplier, display_path.rows * multiplier);
    cv::resize(display_path, temp, size);

    ui->img_label->setPixmap(QPixmap::fromImage(Mat2QImage(temp)));

    _start_point_pub.publish(start_pt);
    _end_point_pub.publish(end_pt);
}

void MainWindow::on_horizontalScrollBar_sliderMoved(int position)
{
//    std::cout << position << std::endl;
}


void MainWindow::on_in_start_x_editingFinished()
{
//    update_inputs();

    cv::Mat show_img = path_img.clone();

    cv::Point2f start_pt;
    cv::Point2f end_pt;

    if(!_start_x)
        start_pt.x = 0;
    else
        start_pt.x = _start_x;

    if(!_start_y)
        start_pt.y = 0;
    else
        start_pt.y = _start_y;

    if(!_end_x)
        end_pt.x = 0;
    else
       end_pt.x = _end_x;

    if(!_end_y)
        end_pt.y = 0;
    else
        end_pt.y = _end_y;



}

void MainWindow::check_callbacks()
{
    ros::spinOnce();
}

void MainWindow::on_bu_clear_clicked()
{
    ui->out_ASTART_error->setText(" ");
    ui->out_MAIN_error->setText(" ");

    _a_start_error.clear();
    _main_path_error.clear();
}

void MainWindow::on_ch_disp_error_clicked(bool checked)
{
    if(checked)
    {
        _error_show = true;
    }
    else
    {
        _error_show = false;
    }
}

void MainWindow::on_bu_shutdown_clicked()
{
    std_msgs::Bool shutdown;
    shutdown.data = true;

    _shutdown_pub.publish(shutdown);

    MainWindow::close();
}
