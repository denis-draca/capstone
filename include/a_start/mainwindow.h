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

    void path_callback(const geometry_msgs::PoseArrayConstPtr &msg);
    cv::Mat path_to_img(geometry_msgs::PoseArray &path);

    void update_inputs();

    void setup_landmarks();

public:
    explicit MainWindow(ros::NodeHandle &n, QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on__bu_find_path_clicked();

    void on_horizontalScrollBar_sliderMoved(int position);

    void on_in_start_x_editingFinished();

    void check_callbacks();


private: // members
    Ui::MainWindow *ui;

    ros::NodeHandle _n;

    cv::Mat path_img;

    int multiplier;

    int _start_x;
    int _start_y;
    int _end_x;
    int _end_y;

    QTimer *_timer;

    ros::Subscriber _path_sub;
    ros::Publisher _start_point_pub;
    ros::Publisher _end_point_pub;


};

#endif // MAINWINDOW_H
