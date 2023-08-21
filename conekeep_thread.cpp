#include "conekeep_thread.h"

void conekeep_thread::run(){

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

    TestMainWindow::rtde_control->setTcp(robot_data::Tcp);

    //设置起始圆锥中心坐标
    robot_data::cone_center = TestMainWindow::rtde_receive->getActualTCPPose();
    double tool_pose[6];
    for (int i = 0; i < 6; i++)
    {
        tool_pose[i] = robot_data::cone_center[i];
    }

    //计算tcp的齐次变换矩阵
    auto T_bt = ur_tcp_pose2matrix4d(tool_pose);

    //计算法兰到tcp的矩阵
    auto T_et = ur_tcp_pose2matrix4d(robot_data::double_tcp);

    Eigen::Matrix4d frange_R = T_bt * T_et.inverse();
    //从tcp得到法兰的中心点 作为圆的中心
    robot_data::frange_fromTcp = matrix4d2rv(frange_R.data());

    while(robot_data::start_conekeep){
       robot_data::wrench_ft_frame << 0,
                                      0,
                                      0,
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

        std::vector<double> cartesian_velocities{robot_data::arm_desired_twist_[0] ,
                                                 robot_data::arm_desired_twist_[1] ,
                                                 robot_data::arm_desired_twist_[2] ,
                                                 robot_data::arm_desired_twist_[3] ,
                                                 robot_data::arm_desired_twist_[4] ,
                                                 robot_data::arm_desired_twist_[5] };
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

        // 圆中心坐标
        Eigen::Vector3d con_center = {robot_data::frange_fromTcp[0],robot_data::frange_fromTcp[1], robot_data::frange_fromTcp[2]};

        Eigen::Vector3d P(actual_frange_fromTcp[0] ,
                          actual_frange_fromTcp[1] ,
                          actual_frange_fromTcp[2] );

        qDebug()<<"点到中心点的距离：."<<ComputePointToPoint(P,con_center);

        double dt = 0.01;
        std::vector<double> new_pose = {0,0,0,0,0,0};
        new_pose[0] = robot_data::g_TCP_Pose[0] + cartesian_velocities[0] * dt;
        new_pose[1] = robot_data::g_TCP_Pose[1] + cartesian_velocities[1] * dt;
        new_pose[2] = robot_data::g_TCP_Pose[2] + cartesian_velocities[2] * dt;
        new_pose[3] = robot_data::g_TCP_Pose[3] + cartesian_velocities[3] * dt*20;
        new_pose[4] = robot_data::g_TCP_Pose[4] + cartesian_velocities[4] * dt*20;
        new_pose[5] = robot_data::g_TCP_Pose[5] + cartesian_velocities[5] * dt*20;


        if(robot_data::start_move==true){
            qDebug()<<robot_data::cone_dist;
            if(ComputePointToPoint(P,con_center)<robot_data::cone_dist){
                TestMainWindow::rtde_control->servoL(new_pose, 1, 1, 1.0/500, 0.1, 300);
            }
            else if(0.045<ComputePointToPoint(P,con_center) && ComputePointToPoint(P,con_center)<0.48){
                TestMainWindow::rtde_control->servoL(new_pose, 1, 1, 1.0/500, 0.1, 300);
                if(ComputePointToPoint(P,con_center)>0.048){
                    robot_data::pose_queue.push(robot_data::g_TCP_Pose);
                }
                if(robot_data::pose_queue.size()>10){
                    robot_data::pose_queue.pop();
                }
            }
            else if(ComputePointToPoint(P,con_center)>0.05){
                TestMainWindow::rtde_control->servoL(robot_data::pose_queue.back(), 1, 1, 1.0/500, 0.1, 300);
        }
        }
        QThread::msleep(10);
    }
}

// tcp转变换矩阵
Eigen::MatrixXd conekeep_thread::ur_tcp_pose2matrix4d(double *ur_tcp_pose){
    auto p = Eigen::Map<Eigen::VectorXd>(ur_tcp_pose, 6); // [x, y, z, rx, ry, rz]
    auto position = Eigen::Vector3d(p.block<3, 1>(0, 0)); // [x, y, z]
    auto rotation = Eigen::Vector3d(p.block<3, 1>(3, 0)); // [rx, ry, rz]
    auto rotation_vector = Eigen::AngleAxisd(rotation.norm(), rotation.normalized());
    Eigen::Matrix4d matrix4d = Eigen::Matrix4d::Identity();
    matrix4d.block<3, 3>(0, 0) = rotation_vector.toRotationMatrix(); // 旋转矩阵
    matrix4d.block<3, 1>(0, 3) = position;                           // 位移
    return matrix4d;
}

// 变换矩阵转旋转角
std::vector<double> conekeep_thread::matrix4d2rv(double *matrix)
{
    Eigen::Matrix4d matrix4d = Eigen::Map<Eigen::MatrixXd>(matrix, 4, 4);
    Eigen::AngleAxisd rotation_vector;
    rotation_vector.fromRotationMatrix(matrix4d.transpose().block<3, 3>(0, 0));
    Eigen::Vector3d axis = rotation_vector.axis();
    double angle = rotation_vector.angle();
    std::vector<double> rv = {matrix[12], matrix[13], matrix[14], axis[0] * angle, axis[1] * angle, axis[2] * angle};
    return rv;
}


// 点到点的距离
double conekeep_thread::ComputePointToPoint(Eigen::Vector3d &P1, Eigen::Vector3d &P2){
        double dx = P1[0] - P2[0];
        double dy = P1[1] - P2[1];
        double dz = P1[2] - P2[2];

        return sqrt(dx * dx + dy * dy + dz * dz);
}




