#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H
#include <iostream>
#include <vector>
#include <QDebug>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <queue>

using namespace Eigen;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

class robot_data
{
public:
    robot_data(){};
    ///
    /// \brief g_TCP_Pose 静态全局变量 TCP位姿
    ///
    static std::vector<double> g_TCP_Pose;
    ///
    /// \brief g_TCP_Speed 静态全局变量 TCP速度
    ///
    static std::vector<double> g_TCP_Speed;
    ///
    /// \brief g_TCP_Force 静态全局变量 TCP力、力矩
    ///
    static std::vector<double> g_TCP_Force;

    ///
    /// \brief M_a_ 惯性矩阵
    ///
    static Matrix6d M_a_;
    ///
    /// \brief D_a_阻尼矩阵
    ///
    static Matrix6d D_a_;

    //double arm_max_vel_ = 0.3;
    static double arm_max_acc_ ;

    ///
    /// \brief 求出的加速度
    ///
    static Vector6d arm_desired_accelaration;

    static Vector6d arm_desired_twist_;

    static Vector6d wrench_external_;

    static Vector6d wrench_ft_frame;

    ///
    /// \brief 自由拖动开关
    ///
    static bool start_freemove;

    ///
    /// \brief 直线保持开关
    ///
    static bool start_linekeep;

    ///
    /// \brief 直线保持的两个点
    ///
    static Vector3d line_point1;
    static Vector3d line_point2;


    ///
    /// \brief 平面保持开关
    ///
    static bool start_planekeep;

    ///
    /// \brief 圆锥保持开关
    ///
    static bool start_conekeep;

    static std::vector<double> cone_center;

    static std::vector<double> Tcp;

    static double double_tcp[6];

    static std::vector<double> frange_fromTcp;

    static std::queue<std::vector<double>> pose_queue;

    ///
    /// \brief 半球保持开关
    ///
    static bool start_ballkeep;

    static Vector3d ball_center;

    ///
    /// \brief 机器人开始读取数据开关
    ///
    static bool start_readdata;

    static std::queue<std::vector<double>> ball_queue;

    static double plane_angle;

    static bool start_move;

    ///
    /// \brief 圆锥保持--点到圆心的距离
    ///
    static double cone_dist;

    ///
    /// \brief 判断是否找到平面
    ///
    static bool start_find_plane;

    ///
    /// \brief 平面对刀开关
    ///
    static bool start_newplanekeep;

    ///
    /// \brief 平面对刀的夹角
    ///
    static double new_plane_angle;

    ///
    /// \brief 三个点到平面的距离
    ///
    static double pt2plane_angle1;
    static double pt2plane_angle2;
    static double pt2plane_angle3;

};




#endif // ROBOT_DATA_H
