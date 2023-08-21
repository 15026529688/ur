#include "linekeep_thread.h"

bool linekeep_thread::PointInLine(const Eigen::Vector3d &pt, const Eigen::Vector3d &begin, const Eigen::Vector3d &end)
{

    Vector3d P1P2;
    P1P2 << begin - end;
    Vector3d P1P;
    P1P << begin - pt;

    if (0 < fabs((P1P2.cross(P1P)).norm()) && fabs((P1P2.cross(P1P)).norm()) < 0.001)
    {
        return true;
    }
    else
    {
        return false;
    }
}


// 点到直线中垂直的点
Eigen::Vector3d linekeep_thread::GetFootOfPerpendicular(const Eigen::Vector3d &pt,
                                                        const Eigen::Vector3d &begin,
                                                        const Eigen::Vector3d &end)
{

    // auto diff = end - begin;
    auto diff = begin - end;
    auto diff_sqr_norm = diff.squaredNorm();
    if (diff_sqr_norm < 0.00000001)
    {
        return begin;
    }

    // double u = ((pt - begin).transpose() * diff)(0) / diff_sqr_norm;;
    double u = ((pt[0] - begin[0]) * (begin[0] - end[0]) +
                (pt[1] - begin[1]) * (begin[1] - end[1]) +
                (pt[2] - begin[2]) * (begin[2] - end[2])) /
               diff_sqr_norm;

    return begin + u * diff;
}



Eigen::Vector3d linekeep_thread::GetFootOfPerpendicular2(const Eigen::Vector3d &pt, const Eigen::Vector3d &begin, const Eigen::Vector3d &end)
// pt:point outside line
{
    auto diff = end - begin;
    auto diff_sqr_norm = diff.squaredNorm();
    if (diff_sqr_norm < 0.00000001)
    {
        return begin;
    }

    double u = ((pt - begin).transpose() * diff)(0) / diff_sqr_norm;

    // cout<<"u: "<<u*diff_sqr_norm<<endl;
    return begin + u * diff;
}

void linekeep_thread::run(){

    robot_data::arm_desired_accelaration.setZero();
    robot_data::wrench_external_.setZero();
    robot_data::arm_desired_twist_.setZero();
    robot_data::wrench_ft_frame.setZero();
    robot_data::M_a_ <<6, 0, 0, 0, 0, 0,
                        0, 6, 0, 0, 0, 0,
                        0, 0, 6, 0, 0, 0,
                        0, 0, 0, 2, 0, 0,
                        0, 0, 0, 0, 2, 0,
                        0, 0, 0, 0, 0, 2;

    robot_data::D_a_ <<40, 0, 0, 0, 0, 0,
                        0, 40, 0, 0, 0, 0,
                        0, 0, 40, 0, 0, 0,
                        0, 0, 0, 15, 0, 0,
                        0, 0, 0, 0, 15, 0,
                        0, 0, 0, 0, 0, 15;

    //添加负载
    std::vector<double> cog = {0,0.02,0};  //质心位置
    TestMainWindow::rtde_control->setPayload(1.1,cog);

    //先移动到预定点
    std::vector<double> prepare_pose = {-0.156, -0.518, 0.346, 3.124, 0.422, -0.1};
    TestMainWindow::rtde_control->moveL(prepare_pose);
    std::vector<double> Pnow = TestMainWindow::rtde_receive->getActualTCPPose();
    Eigen::Vector3d  P_now (Pnow[0],Pnow[1],Pnow[2]);

    //判断当前点是否在直线上
    if(PointInLine(P_now,robot_data::line_point1,robot_data::line_point2) == false){
        //计算当前点到线段的最近距离
        Eigen::Vector3d  near_pt = GetFootOfPerpendicular(P_now,robot_data::line_point1,robot_data::line_point2);
        //qDebug()<<"最近的点："<<near_pt[0]<<endl;

        std::vector<double> near_point = {near_pt[0], near_pt[1], near_pt[2], Pnow[3], Pnow[4], Pnow[5]};
        TestMainWindow::rtde_control->moveL(near_point);
    }

    while(robot_data::start_linekeep){

        //读取机械臂的值
       robot_data::wrench_ft_frame <<  robot_data::g_TCP_Force[0],
                                       robot_data::g_TCP_Force[1],
                                       robot_data::g_TCP_Force[2],
                                       robot_data::g_TCP_Force[3],
                                       robot_data::g_TCP_Force[4],
                                       robot_data::g_TCP_Force[5];

       robot_data::wrench_external_ << robot_data::wrench_ft_frame;

       robot_data::arm_desired_accelaration = robot_data::M_a_.inverse() *
                            (-robot_data::D_a_ * robot_data::arm_desired_twist_ +
                             robot_data::wrench_external_);

        // limiting the accelaration for better stability and safety
        double a_acc_norm = (robot_data::arm_desired_accelaration.segment(0, 3)).norm();
        if (a_acc_norm > robot_data::arm_max_acc_)
        {
            robot_data::arm_desired_accelaration.segment(0, 3) *=
                    (robot_data::arm_max_acc_ / a_acc_norm);
        }

        robot_data::arm_desired_twist_ += robot_data::arm_desired_accelaration * 0.1;

        std::vector<double> cartesian_velocities{robot_data::arm_desired_twist_[0]  ,
                                                 robot_data::arm_desired_twist_[1]  ,
                                                 robot_data::arm_desired_twist_[2]  ,
                                                 robot_data::arm_desired_twist_[3]  ,
                                                 robot_data::arm_desired_twist_[4]  ,
                                                 robot_data::arm_desired_twist_[5]  };

        double dt = 0.01;
        std::vector<double> new_pose = {0,0,0,0,0,0};
        new_pose[0] = robot_data::g_TCP_Pose[0] + cartesian_velocities[0] * dt;
        new_pose[1] = robot_data::g_TCP_Pose[1] + cartesian_velocities[1] * dt;
        new_pose[2] = robot_data::g_TCP_Pose[2] + cartesian_velocities[2] * dt;
        new_pose[3] = robot_data::g_TCP_Pose[3] + cartesian_velocities[3] * dt;
        new_pose[4] = robot_data::g_TCP_Pose[4] + cartesian_velocities[4] * dt;
        new_pose[5] = robot_data::g_TCP_Pose[5] + cartesian_velocities[5] * dt;

        Eigen::Vector3d P(new_pose[0],new_pose[1],new_pose[2]);
        //求点到直线的垂点
        Eigen::Vector3d Pp = GetFootOfPerpendicular2(P,robot_data::line_point1,
                                                     robot_data::line_point2);

        new_pose = {Pp[0],Pp[1],Pp[2],new_pose[3],new_pose[4],new_pose[5]};
        if(robot_data::start_move==true){
            TestMainWindow::rtde_control->servoL(new_pose, 1, 1, 1.0/500, 0.1, 300);
        }
        else if(robot_data::start_move ==false){

        }
//        Eigen::Vector3d P(robot_data::g_TCP_Pose[0]+robot_data::arm_desired_twist_[0] ,
//                        robot_data::g_TCP_Pose[1]+robot_data::arm_desired_twist_[1] ,
//                        robot_data::g_TCP_Pose[2]+robot_data::arm_desired_twist_[2] );

//        //求点到直线的垂点
//        Eigen::Vector3d Pp = GetFootOfPerpendicular2(P,robot_data::line_point1,
//                                                     robot_data::line_point2);

//        std::vector<double> cv = {Pp[0]-robot_data::g_TCP_Pose[0],
//                                Pp[1]-robot_data::g_TCP_Pose[1],
//                                Pp[2]-robot_data::g_TCP_Pose[2],
//                                cartesian_velocities[3],
//                                cartesian_velocities[4],
//                                cartesian_velocities[5]};

//        for(int i =0;i<6;i++){
//            qDebug()<<cv[i];
//        }

//        MainWindow::rtde_control->speedL(cv, 0.1, 0.1);
        QThread::msleep(10);
     }

}

