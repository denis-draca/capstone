#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QGraphicsPixmapItem>
#include <iostream>
#include <QGraphicsScene>
#include <QWheelEvent>
#include <geometry_msgs/Point.h>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include <QValidator>
#include <QTimer>
#include "geometry_msgs/PoseArray.h"
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <fstream>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

private: //methods
    explicit MainWindow(QWidget *parent = 0);

    QImage Mat2QImage(cv::Mat const& src);
    cv::Mat QImage2Mat(QImage const& src);
    cv::Mat resize_to_multipler(cv::Mat &small_img);

    void path_callback(const geometry_msgs::PoseArrayConstPtr &msg);
    cv::Mat path_to_img(geometry_msgs::PoseArray &path);

    void update_inputs();

    void setup_landmarks();

    void display_landmarks();

    void a_start_error_callback(const std_msgs::String &msg);
    void main_path_error_callback(const std_msgs::String &msg);
    void directions_callback(const std_msgs::String &msg);
    void direction_pts_callback(const geometry_msgs::PoseArrayConstPtr &msg);

public:
    explicit MainWindow(ros::NodeHandle &n, QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on__bu_find_path_clicked();

    void on_horizontalScrollBar_sliderMoved(int position);

    void on_in_start_x_editingFinished();

    void check_callbacks();


    void on_bu_clear_clicked();

private slots:
    void on_checkBox_clicked(bool checked);
    void on_ch_disp_error_clicked(bool checked);

    void on_bu_shutdown_clicked();

    void on_bu_set_dir_clicked();

    void on_bu_screenshot_aStar_clicked();

    void on_bu_set_dir_directions_clicked();

    void on_bu_screenshot_directions_clicked();

    void on_bu_user_submit_clicked();

private: // members
    Ui::MainWindow *ui;

    ros::NodeHandle _n;

    cv::Mat path_img;
    cv::Mat _resized_a_star;
    cv::Mat _landmark_map;
    cv::Mat _directions_map;

    int multiplier;

    int _start_x;
    int _start_y;
    int _end_x;
    int _end_y;

    QTimer *_timer;

    ros::Subscriber _path_sub;
    ros::Subscriber _error_a_start_sub;
    ros::Subscriber _error_main_sub;
    ros::Subscriber _path_directions_sub;
    ros::Subscriber _path_points_sub;

    ros::Publisher _start_point_pub;
    ros::Publisher _end_point_pub;
    ros::Publisher _shutdown_pub;

    std::vector<std::pair<int,int>> _landmarks;

    std::string _a_start_error;
    std::string _main_path_error;
    std::string _direction_list;

    QString _astar_screenshot_dir;
    QString _directions_screenshot_dir;
    QString _landmarks_screenshot_dir;

    bool _error_show;



    bool _display_err;

};

#endif // MAINWINDOW_H
