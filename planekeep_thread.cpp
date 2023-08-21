#include "planekeep_thread.h"

void planekeep_thread::run(){
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
                        0, 0, 0, 10, 0, 0,
                        0, 0, 0, 0, 10, 0,
                        0, 0, 0, 0, 0, 10;

    //添加负载
    std::vector<double> cog = {0,0.2,0};  //质心位置
    TestMainWindow::rtde_control->setPayload(0.8,cog);

    //先移动到预定点
//    std::vector<double> prepare_pose = {-0.156, -0.518, 0.346, 3.124, 0.422, -0.1};
//    MainWindow::rtde_control->moveL(prepare_pose);
//    std::vector<double> Pnow = MainWindow::rtde_receive->getActualTCPPose();
//    Eigen::Vector3d  P_now (Pnow[0],Pnow[1],Pnow[2]);

    //移动到平面上
    // 平面保持测试点
    Eigen::Vector3d plane_p1(-0.1157, -0.6287, 0.2253);
    Eigen::Vector3d plane_p2(-0.127, -0.35, 0.4794);
    Eigen::Vector3d plane_p3(-0.36, -0.35, 0.4846);
    // D(x2-x1+x3,y2-y1+y3,z2-z1+z3)
    Eigen::Vector3d plane_p4(-0.45, -0.6287, 0.2305);

    //Eigen::Vector3d line1 = plane_p3 - plane_p2;

    // 三点求平面方程
    Eigen::Vector4d plane_coeff = getPlaneBy3Points(plane_p1,plane_p2,plane_p3);
    qDebug()<<"plane_coeff:"<<plane_coeff[0]<<plane_coeff[1]<<plane_coeff[2]<<plane_coeff[3];

    //设置tcp
    std::vector<double> test_tcp ={0,0,0.02,0,0,0};
    TestMainWindow::rtde_control->setTcp(test_tcp);

    //先判断平面是否平行
    //机器人的平面
    Eigen::Vector4d robot_plane_coeff;
    double actual_tcp[6];
    for (int j = 0; j < 6; j++)
    {
        actual_tcp[j] =robot_data::g_TCP_Pose[j];
    }
    auto T_actual_tcp = ur_tcp_pose2matrix4d(actual_tcp);
    auto T_et = ur_tcp_pose2matrix4d(robot_data::double_tcp);
    Eigen::Matrix4d actual_frange_R = T_actual_tcp * T_et.inverse();
    // TCP坐标转法兰坐标
    std::vector<double> actual_frange_fromTcp = matrix4d2rv(actual_frange_R.data());
    Eigen::Vector3d robot_plane1(robot_data::g_TCP_Pose[0],robot_data::g_TCP_Pose[1],robot_data::g_TCP_Pose[2]);
    Eigen::Vector3d robot_plane2(actual_frange_fromTcp[0],actual_frange_fromTcp[1],actual_frange_fromTcp[2]);
    Eigen::Vector3d robot_plane3(robot_data::g_TCP_Pose[0]-0.1,robot_data::g_TCP_Pose[1],robot_data::g_TCP_Pose[2]);
    robot_plane_coeff = getPlaneBy3Points(robot_plane1,robot_plane2,robot_plane3);
    robot_data::plane_angle = get_angle(plane_coeff,robot_plane_coeff);

    if(robot_data::plane_angle<2){
        robot_data::start_find_plane = true;
    }
    else {
        robot_data::start_find_plane = false;
    }

    //开始循环
    while(robot_data::start_planekeep==true){

        if(robot_data::start_find_plane == false){
           //读取机械臂的值
           robot_data::wrench_ft_frame <<robot_data::g_TCP_Force[0],
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
                qDebug() << "Admittance generates high arm accelaration!"
                     << " norm: " << a_acc_norm;

                robot_data::arm_desired_accelaration.segment(0, 3) *=
                        (robot_data::arm_max_acc_ / a_acc_norm);
            }

            robot_data::arm_desired_twist_ += robot_data::arm_desired_accelaration * 0.1;

            //qDebug()<<"arm_desired_twist_ "<<robot_data::arm_desired_twist_[0];

            std::vector<double> cartesian_velocities{robot_data::arm_desired_twist_[0] ,
                                                     robot_data::arm_desired_twist_[1] ,
                                                     robot_data::arm_desired_twist_[2] ,
                                                     robot_data::arm_desired_twist_[3] ,
                                                     robot_data::arm_desired_twist_[4] ,
                                                     robot_data::arm_desired_twist_[5] };

            Eigen::Vector3d P(robot_data::g_TCP_Pose[0] ,
                              robot_data::g_TCP_Pose[1] ,
                              robot_data::g_TCP_Pose[2] );

            //pointinplane(P,plane_coeff);

            // 计算实时的法兰pose
            double actual_tcp[6];
            for (int j = 0; j < 6; j++)
            {
                actual_tcp[j] =robot_data::g_TCP_Pose[j];
            }
            auto T_actual_tcp = ur_tcp_pose2matrix4d(actual_tcp);

            auto T_et = ur_tcp_pose2matrix4d(robot_data::double_tcp);

            Eigen::Matrix4d actual_frange_R = T_actual_tcp * T_et.inverse();
            // TCP坐标转法兰坐标
            std::vector<double> actual_frange_fromTcp = matrix4d2rv(actual_frange_R.data());

            //求解机械臂的平面
            //std::vector<double> robot_plane_pose = MainWindow::rtde_receive->getActualTCPPose();
            Eigen::Vector3d robot_plane1(robot_data::g_TCP_Pose[0],robot_data::g_TCP_Pose[1],robot_data::g_TCP_Pose[2]);
            Eigen::Vector3d robot_plane2(actual_frange_fromTcp[0],actual_frange_fromTcp[1],actual_frange_fromTcp[2]);
            Eigen::Vector3d robot_plane3(robot_data::g_TCP_Pose[0]-0.1,robot_data::g_TCP_Pose[1],robot_data::g_TCP_Pose[2]);
            robot_plane_coeff = getPlaneBy3Points(robot_plane1,robot_plane2,robot_plane3);

    //        qDebug()<<"robot_plane1"<<robot_plane1[0];
    //        qDebug()<<"robot_plane_coeff:"<<robot_plane_coeff[0]<<robot_plane_coeff[1]<<robot_plane_coeff[2]<<robot_plane_coeff[3];

            //qDebug()<<get_angle(plane_coeff,robot_plane_coeff);
            robot_data::plane_angle = get_angle(plane_coeff,robot_plane_coeff);

            double dt = 0.01;
            std::vector<double> new_pose = {0,0,0,0,0,0};
            new_pose[0] = robot_data::g_TCP_Pose[0] + cartesian_velocities[0] * dt*2;
            new_pose[1] = robot_data::g_TCP_Pose[1] + cartesian_velocities[1] * dt*2;
            new_pose[2] = robot_data::g_TCP_Pose[2] + cartesian_velocities[2] * dt*2;
            new_pose[3] = robot_data::g_TCP_Pose[3] + cartesian_velocities[3] * dt*30;
            new_pose[4] = robot_data::g_TCP_Pose[4] + cartesian_velocities[4] * dt*30;
            new_pose[5] = robot_data::g_TCP_Pose[5] + cartesian_velocities[5] * dt*30;

            if(robot_data::start_move==true){
                TestMainWindow::rtde_control->servoL(new_pose, 1, 1, 1.0/500, 0.1, 300);
                if(robot_data::plane_angle<2){
                    robot_data::start_find_plane=true;
                }
            }
            else if(robot_data::start_move ==false){

            }


        }

        //两个平面重合
        if(robot_data::start_find_plane==true)  //&& fabs(plane_coeff[4]-robot_plane_coeff[4]<0.1
        {
//            //qDebug()<<"平面重合";

//            robot_data::wrench_ft_frame << 0,
//                                           0,
//                                           0,
//                                           0,
//                                           robot_data::g_TCP_Force[4],
//                                           robot_data::g_TCP_Force[5];

//            robot_data::wrench_external_ << robot_data::wrench_ft_frame;

//            robot_data::arm_desired_accelaration = robot_data::M_a_.inverse() *
//                                 (-robot_data::D_a_ * robot_data::arm_desired_twist_ +
//                                  robot_data::wrench_external_);

//             // limiting the accelaration for better stability and safety
//             double a_acc_norm = (robot_data::arm_desired_accelaration.segment(0, 3)).norm();
//             if (a_acc_norm > robot_data::arm_max_acc_)
//             {
//                 // 没带频率
//                 qDebug() << "Admittance generates high arm accelaration!"
//                      << " norm: " << a_acc_norm;

//                 robot_data::arm_desired_accelaration.segment(0, 3) *=
//                         (robot_data::arm_max_acc_ / a_acc_norm);
//             }

//             robot_data::arm_desired_twist_ += robot_data::arm_desired_accelaration * 0.1;


////             // 计算实时的法兰pose
////             double actual_tcp[6];
////             for (int j = 0; j < 6; j++)
////             {
////                 actual_tcp[j] =robot_data::g_TCP_Pose[j];
////             }
////             auto T_actual_tcp = ur_tcp_pose2matrix4d(actual_tcp);

////             auto T_et = ur_tcp_pose2matrix4d(robot_data::double_tcp);

////             Eigen::Matrix4d actual_frange_R = T_actual_tcp * T_et.inverse();
////             // TCP坐标转法兰坐标
////             std::vector<double> actual_frange_fromTcp = matrix4d2rv(actual_frange_R.data());

////             //求解机械臂的平面
////             //std::vector<double> robot_plane_pose = MainWindow::rtde_receive->getActualTCPPose();
//             Eigen::Vector3d robot_plane1(robot_data::g_TCP_Pose[0],robot_data::g_TCP_Pose[1],robot_data::g_TCP_Pose[2]);
////             Eigen::Vector3d robot_plane2(actual_frange_fromTcp[0],actual_frange_fromTcp[1],actual_frange_fromTcp[2]);
//             Eigen::Vector3d robot_plane3(robot_data::g_TCP_Pose[0]-0.1,robot_data::g_TCP_Pose[1],robot_data::g_TCP_Pose[2]);
////             robot_plane_coeff = getPlaneBy3Points(robot_plane1,robot_plane2,robot_plane3);

//             Eigen::Vector3d line2 = robot_plane3 -robot_plane1;

//             double cosValNew = line1.dot(line2) / (line1.norm()*line2.norm());
//             double angleNew = acos(cosValNew) * 180 / M_PI;

//             qDebug()<<"angleNew"<<angleNew;

//             std::vector<double> cartesian_velocities{robot_data::arm_desired_twist_[0] ,
//                                                      robot_data::arm_desired_twist_[1] ,
//                                                      robot_data::arm_desired_twist_[2] ,
//                                                      robot_data::arm_desired_twist_[3] ,
//                                                      robot_data::arm_desired_twist_[4] ,
//                                                      robot_data::arm_desired_twist_[5] };
//             MainWindow::rtde_control->speedL(cartesian_velocities, 0.1, 0.1);
            //qDebug()<<cartesian_velocities[0]<<cartesian_velocities[1]<<cartesian_velocities[2];

                //平面保持模块

                //读取机械臂的值
               robot_data::wrench_ft_frame <<robot_data::g_TCP_Force[0],
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
                    qDebug() << "Admittance generates high arm accelaration!"
                         << " norm: " << a_acc_norm;

                    robot_data::arm_desired_accelaration.segment(0, 3) *=
                            (robot_data::arm_max_acc_ / a_acc_norm);
                }

                robot_data::arm_desired_twist_ += robot_data::arm_desired_accelaration * 0.1;

                //qDebug()<<"arm_desired_twist_ "<<robot_data::arm_desired_twist_[0];

                std::vector<double> cartesian_velocities{robot_data::arm_desired_twist_[0] ,
                                                         robot_data::arm_desired_twist_[1] ,
                                                         robot_data::arm_desired_twist_[2] ,
                                                         robot_data::arm_desired_twist_[3] ,
                                                         robot_data::arm_desired_twist_[4] ,
                                                         robot_data::arm_desired_twist_[5] };

                double dt = 0.01;
                std::vector<double> new_pose = {0,0,0,0,0,0};
                new_pose[0] = robot_data::g_TCP_Pose[0] + cartesian_velocities[0] * dt;
                new_pose[1] = robot_data::g_TCP_Pose[1] + cartesian_velocities[1] * dt;
                new_pose[2] = robot_data::g_TCP_Pose[2] + cartesian_velocities[2] * dt;
                new_pose[3] = robot_data::g_TCP_Pose[3] + cartesian_velocities[3] * dt;
                new_pose[4] = robot_data::g_TCP_Pose[4] + cartesian_velocities[4] * dt;
                new_pose[5] = robot_data::g_TCP_Pose[5] + cartesian_velocities[5] * dt;

                Eigen::Vector3d P(robot_data::g_TCP_Pose[0] ,
                                  robot_data::g_TCP_Pose[1] ,
                                  robot_data::g_TCP_Pose[2] );

                pointinplane(P,plane_coeff);
                std::vector<double> ptemp;
                double A = plane_coeff[0];
                double B = plane_coeff[1];
                double C = plane_coeff[2];
                double D = plane_coeff[3];

                double ABC = A*A + B*B + C*C;
                qDebug()<<A*A + B*B + C*C;
                double cmd1;
                double cmd2;
                double cmd3;
                //cmd = (B*B + C*C)*cartesian_velocities[0];

                //只能接受double  vector<double>不行
                cmd1 = ((B*B + C*C)*new_pose[0] - A*(B*new_pose[1] + C*new_pose[2] + D))/ABC;
                cmd2 = ((A*A + C*C)*new_pose[1] - B*(A*new_pose[0] + C*new_pose[2] + D))/ABC ;
                cmd3 = ((A*A + B*B)*new_pose[2] - C*(A*new_pose[0] + B*new_pose[1] + D))/ABC;

                std::vector<double> cv = {cmd1,cmd2,cmd3,new_pose[3],new_pose[4],new_pose[5]};
                if(robot_data::start_move==true){
                    qDebug()<<new_pose[0]<<new_pose[1]<<new_pose[2];
                    TestMainWindow::rtde_control->servoL(cv, 1, 1, 1.0/500, 0.1, 300);
                }
                else if(robot_data::start_move ==false){

                }

                //qDebug()<<cartesian_velocities[0]<<cartesian_velocities[1]<<cartesian_velocities[2];
                //MainWindow::rtde_control->speedL(cv, 0.1, 0.1);
        }

        QThread::msleep(10);
     }
    while (robot_data::start_planekeep == false) {
        return;
    }

}

Eigen::Vector4d planekeep_thread::getPlaneBy3Points(
    const Eigen::Vector3d &P1,
    const Eigen::Vector3d &P2,
    const Eigen::Vector3d &P3)
{
    Eigen::Vector3d V1 = P1 - P2;
    Eigen::Vector3d V2 = P1 - P3;
    Eigen::Vector3d normal;
    Eigen::Vector4d plane_coeff;
    normal = V1.cross(V2);
    plane_coeff.block<3, 1>(0, 0) = normal;
    plane_coeff(3) = -plane_coeff(0) * P1(0) - plane_coeff(1) * P1(1) - plane_coeff(2) * P1(2);
    //std::cout << "plane_coeff:" << plane_coeff.transpose() << std::endl;
    return plane_coeff;
}

//判断点是否在平面上
bool planekeep_thread::pointinplane(Eigen::Vector3d &Point, Eigen::Vector4d &Plane){
    double dist = (Plane[0]*Point[0]+Plane[1]*Point[1]+Plane[2]*Point[2]+Plane[3])/
            sqrt(pow(Plane[0], 2) + pow(Plane[1], 2) + pow(Plane[2], 2)); ;
    //qDebug()<<dist;
    if(dist<0.01){
        return true;
    }
    else {
        return false;
    }
}


//获得两个平面的夹角
double planekeep_thread::get_angle(Eigen::Vector4d n1, Eigen::Vector4d n2){
    double cosθ = abs(n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2]) /
        (sqrt(n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2]) * sqrt(n2[0] * n2[0] + n2[1] * n2[1] + n2[2] * n2[2]));
    double angle = std::acos(cosθ);

    //return angle;
    return angle* 180.0 / 3.1415;
}


// tcp转变换矩阵
Eigen::MatrixXd planekeep_thread::ur_tcp_pose2matrix4d(double *ur_tcp_pose){
    auto p = Eigen::Map<Eigen::VectorXd>(ur_tcp_pose, 6); // [x, y, z, rx, ry, rz]
    auto position = Eigen::Vector3d(p.block<3, 1>(0, 0)); // [x, y, z]
    auto rotation = Eigen::Vector3d(p.block<3, 1>(3, 0)); // [rx, ry, rz]
    // rotation.norm() == sqrt(rx * rx + ry * ry + rz * rz)
    // rotation.normalized() ==[rx, ry, rz] / rotation.norm()
    auto rotation_vector = Eigen::AngleAxisd(rotation.norm(), rotation.normalized());
    Eigen::Matrix4d matrix4d = Eigen::Matrix4d::Identity();
    matrix4d.block<3, 3>(0, 0) = rotation_vector.toRotationMatrix(); // 旋转矩阵
    matrix4d.block<3, 1>(0, 3) = position;                           // 位移

    // std::cout<<"===========ur_tcp_pose2matrix4d================"<<std::endl;
    // std::cout<<matrix4d<<std::endl;
    return matrix4d;
}


// 变换矩阵转旋转角
std::vector<double> planekeep_thread::matrix4d2rv(double *matrix)
{
    Eigen::Matrix4d matrix4d = Eigen::Map<Eigen::MatrixXd>(matrix, 4, 4);
    Eigen::AngleAxisd rotation_vector;
    rotation_vector.fromRotationMatrix(matrix4d.transpose().block<3, 3>(0, 0));
    Eigen::Vector3d axis = rotation_vector.axis();
    double angle = rotation_vector.angle();
    std::vector<double> rv = {matrix[12], matrix[13], matrix[14], axis[0] * angle, axis[1] * angle, axis[2] * angle};
    return rv;
}
