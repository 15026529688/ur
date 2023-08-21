#ifndef TESTMAINWINDOW_H
#define TESTMAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <QDebug>
#include "robot_data.h"
#include "freemove_thread.h"
#include "linekeep_thread.h"
#include "planekeep_thread.h"
#include "conekeep_thread.h"
#include "ballkeep_thread.h"
#include "newplanekeep_thread.h"
#include <QFuture>
#include <QFutureWatcher>
#include <QtConcurrent>
#include <QTimer>
#include <QButtonGroup>

using namespace Eigen;
using namespace std;
using namespace ur_rtde;
using namespace std::chrono;


QT_BEGIN_NAMESPACE
namespace Ui { class TestMainWindow;}
QT_END_NAMESPACE

class TestMainWindow : public QMainWindow
{
    Q_OBJECT


public:
    TestMainWindow(QWidget *parent = nullptr);
    ~TestMainWindow();

    virtual void timerEvent(QTimerEvent *event);
    void f_GetRobotData();
    QTimer mTimer;

    void  stop_robot();
    void  compute_admittance();
    void  send_commands_to_robot();
    void  KeepStraight();
    void  PlaneKeep();
    void  ConeKeep();
    void  thread();
    void  new_plane_keep();
 
    static RTDEControlInterface* rtde_control;
    static RTDEReceiveInterface* rtde_receive;

public slots:
    void  connect_robot();
    void  get_Robotdata();
    void  freemode();
    void  StartTimer();
    void  futureFinished();
    void  ball_keep();
    void  freedrive();
    void  freedrive_close();

signals:
    void sig_fun();
    void started();
    void finished();
    void start_freeMove();
    void start_linekeep();
    void start_planekeep();
    void start_conekeep();
    void start_ballkeep();
    void start_newplanekeep();

private slots:

    void on_initP_btn_clicked();

    void on_base_clockwise_btn_clicked();

    void on_base_counterclockwise_btn_clicked();

    void on_base_counterclockwise_btn_2_clicked();

    void on_Z_clockwise_btn_clicked();

    void on_Rx_clockwise_btn_clicked();

    void on_Rx_counterclockwise_btn_clicked();

    void on_pushButton_clicked();

private:

    Ui::TestMainWindow *ui;

    QPushButton *connect_btn;

    //机器人上电情况
    bool robot_status;

    bool robot_run;

    int mTimerId = -1;
    QThread *new_thread;


    //随动测试用
    std::vector<double> start_circle_pose = {0,0,0,0,0,0};

    double dist_start_center = 0;

    double angle = 0;

    std::vector<double > Z_position ={0,0,0,0,0,0};

    double theta = 1.57;
};
#endif // MAINWINDOW_H
