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


    //Directory member setup
    _astar_screenshot_dir = "/home/denis/catkin_ws/src/a_start/for Report";
    _directions_screenshot_dir = "/home/denis/catkin_ws/src/a_start/for Report";
    _landmarks_screenshot_dir = "/home/denis/catkin_ws/src/a_start/for Report";
}

cv::Mat MainWindow::QImage2Mat(const QImage &src)
{
//    cv::Mat tmp(src.height(), src.width(), CV_8UC, (uchar*)src.bits(), src.bytesPerLine());
//    cv::Mat result;
//    cvtColor(tmp, result, CV_BGR2RGB);

    //    return result;
}

cv::Mat MainWindow::resize_to_multipler(cv::Mat &small_img)
{
    int scale_factor = multiplier;
    cv::Mat larger_img(small_img.rows * scale_factor, small_img.cols*scale_factor, CV_8UC3, cv::Scalar(0,0,0));

    for(int y = 0; y < small_img.rows; y++)
    {
        for(int x = 0; x < small_img.cols; x++)
        {
            cv::Vec3b cell = small_img.at<cv::Vec3b>(y,x);

            for(int z = (y*scale_factor); z < (y*scale_factor) + (scale_factor); z++)
            {
                for(int t = (x*scale_factor) ; t < (x*scale_factor) + (scale_factor); t++)
                {
                    if(z >= 0 && z < larger_img.rows && t >=0 && t < larger_img.cols)
                    {
                        larger_img.at<cv::Vec3b>(z,t) = cell;
                    }
                }
            }
        }
    }

    return larger_img;



}

void MainWindow::path_callback(const geometry_msgs::PoseArrayConstPtr &msg)
{

    geometry_msgs::PoseArray derivedMsg = *msg;

    cv::Mat display_path = path_to_img(derivedMsg);

    _resized_a_star = resize_to_multipler(display_path);

    ui->img_label->setPixmap(QPixmap::fromImage(Mat2QImage(_resized_a_star)));
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

        cv::circle(landmark_disp, pt, 1, cv::Scalar(50, 100, 200));
    }

    _landmark_map = resize_to_multipler(landmark_disp);

    ui->img_landmarks->setPixmap(QPixmap::fromImage(Mat2QImage(_landmark_map)));

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
    _direction_list = msg.data;
    ui->out_directions->setText(_direction_list.c_str());
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

    _directions_map = resize_to_multipler(img);
    ui->img_directions->setPixmap(QPixmap::fromImage(Mat2QImage(_directions_map)));

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

    bool unset = false;
    if(!_start_x)
        unset = true;
//        start_pt.x = 0;
    else
        start_pt.x = _start_x;

    if(!_start_y)
        unset = true;
//        start_pt.y = 0;
    else
        start_pt.y = _start_y;

    if(!_end_x)
        unset = true;
//        end_pt.x = 0;
    else
       end_pt.x = _end_x;

    if(!_end_y)
        unset = true;
//        end_pt.y = 0;
    else
        end_pt.y = _end_y;

    if(unset)
    {
        QMessageBox::information(this, tr("Direction Generator"), tr("No START or END point given, please make sure to provide both"));
        return;
    }

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

void MainWindow::on_checkBox_clicked(bool checked)
{
    if(checked)
        _display_err = true;
    else
        _display_err = false;
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


void MainWindow::on_bu_set_dir_clicked()
{
    _astar_screenshot_dir = QFileDialog::getExistingDirectory(this, "set A* screenshot path");
}

void MainWindow::on_bu_screenshot_aStar_clicked()
{
    std::string file_name = ui->in_screenshot->text().toUtf8().constData();
    std::string dir = _astar_screenshot_dir.toUtf8().constData();

    if(file_name.empty())
        ui->out_ASTART_error->setText("No name given for screenshot");

    if(_resized_a_star.empty())
        ui->out_ASTART_error->setText("No image to take a screenshot of");

    dir.append("/");
    dir.append(file_name.c_str());
    dir.append(".jpg");

    cv::imwrite(dir.c_str(), _resized_a_star);

}

void MainWindow::on_bu_set_dir_directions_clicked()
{
    _directions_screenshot_dir = QFileDialog::getExistingDirectory(this, "Set direction screenshot path");
}

void MainWindow::on_bu_screenshot_directions_clicked()
{
    std::string file_name = ui->in_screenshot_directions->text().toUtf8().constData();
    std::string dir = _directions_screenshot_dir.toUtf8().constData();

    if(file_name.empty())
        ui->out_MAIN_error->setText("No name given for screenshot");

    if(_resized_a_star.empty())
        ui->out_MAIN_error->setText("No image to take a screenshot of");

    dir.append("/");
    dir.append(file_name.c_str());

    std::string directions_img_dir = dir;
    std::string directions_path_dir = dir;
    std::string raw_path = dir;

    directions_img_dir.append(".jpg");
    directions_path_dir.append("(string).txt");
    raw_path.append("(raw_path).jpg");


    cv::imwrite(directions_img_dir.c_str(), _directions_map);
    cv::imwrite(raw_path.c_str(), _resized_a_star);

    std::ofstream file;
    file.open(directions_path_dir.c_str());

    file << _direction_list;

    file.close();


}

void MainWindow::on_bu_user_submit_clicked()
{
    std::string name = ui->in_user_name->text().toUtf8().constData();
    std::string directions = ui->in_user_direction->toPlainText().toUtf8().constData();
    std::string feedback = ui->in_user_direction_feedback->toPlainText().toUtf8().constData();

    if(name.empty())
    {
        QMessageBox::information(this, tr("Direction Generator"), tr("I would like to know your name"));
        return;
    }

    if(directions.empty() || directions == "Please input some text")
    {
        QMessageBox::information(this, tr("Direction Generator"), tr("No User Directions Given! Please provide some"));
        return;
    }

    if(feedback.empty() || feedback == "Please input some text")
    {
        QMessageBox::information(this, tr("Direction Generator"), tr("No Feedback Given! Please provide some."));
        return;
    }
    std::string location = QFileDialog::getExistingDirectory(this, "Save user input").toUtf8().constData();

    if(location.empty())
    {
        QMessageBox::information(this, tr("Direction Generator"), tr("No save location given!!"));
        return;
    }

    std::ofstream file;

    location.append("/");
    location.append(name);

    if(!QDir().exists(location.c_str()))
        QDir().mkdir(location.c_str());

    std::string untainted_location = "Data has been saved to: \n";
    untainted_location.append(location);

    location.append("/");
    location.append(name);

    std::string raw_path = location;
    std::string direction_path = location;
    std::string direction_list = location;
    std::string landmarks = location;

    raw_path.append("(raw_path).jpg");
    direction_path.append("(direction_path).jpg");
    direction_list.append("(listed_paths).txt");
    landmarks.append("(landmarks).jpg");

    cv::imwrite(raw_path.c_str(), _resized_a_star);
    cv::imwrite(direction_path.c_str(), _directions_map);
    cv::imwrite(landmarks.c_str(), _landmark_map);


    location.append(".txt");

    file.open(location);

    file << "Name: " << name << std::endl;

    file << "start (x,y): " << "(" << _start_x << "," << _start_y << ")" << std::endl;
    file << "end (x,y): " << "(" << _end_x << "," << _end_y << ")" << std::endl;

    file << "USER INSTRUCTIONS START HERE -----------" << std::endl;

    file << directions;

    file << std::endl << std::endl << std::endl;


    file << "USER FEEDBACK STARTS HERE --------------" << std::endl;

    file << feedback;


    file.close();


    file.open(direction_list);

    file << _direction_list;
    file.close();

    QMessageBox::information(this, tr("Direction Generator"), tr(untainted_location.c_str()));

}

void MainWindow::on_slide_shifter_sliderMoved(int position)
{
//    ui->out_MAIN_error->setText(std::to_string(position).c_str());

    double percentage = ((double)position)/100.0;
    bool x_checked = ui->ch_shift_end_X->isChecked();
    bool y_checked = ui->ch_shift_end_y->isChecked();


    if(x_checked)
    {
        int value = path_img.cols * percentage;

        ui->in_end_x->setText(std::to_string(value).c_str());
    }

    if(y_checked)
    {
        int value = path_img.rows * percentage;

        ui->in_end_y->setText(std::to_string(value).c_str());
    }

    on__bu_find_path_clicked();

}
