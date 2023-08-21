#include "my_thread.h"
#include <QThread>
#include <QDebug>
#include <iostream>
#include "testmainwindow.h"

using namespace std;

my_thread::my_thread(QObject *parent):QThread(parent)
{
}

void my_thread::fun()
{

    while(robot_data::start_readdata==true)
    {
        // 位姿获取
        robot_data::g_TCP_Pose = TestMainWindow::rtde_receive->getActualTCPPose();
        // 速度获取
        robot_data::g_TCP_Speed  = TestMainWindow::rtde_receive->getActualTCPSpeed();
        // 力力矩获取
        robot_data::g_TCP_Force = TestMainWindow::rtde_receive->getActualTCPForce();

        if(abs(robot_data::g_TCP_Force[0])>2 || abs(robot_data::g_TCP_Force[1])>2||abs(robot_data::g_TCP_Force[2])>2)
        {
            robot_data::start_move = true;
        }
        else if(abs(robot_data::g_TCP_Force[0])<1 || abs(robot_data::g_TCP_Force[1])<1||abs(robot_data::g_TCP_Force[2])<1)
        {
            robot_data::start_move = false;
        }

        //qDebug()<<"拖动开关: ."<<robot_data::start_move;
        QThread::msleep(20);
    }
}


void my_thread::run()
{
    fun();
}
