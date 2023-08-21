#include "newplanekeep_thread.h"

/**
 * @brief 平面拖动寻找平面
 */

newplanekeep_thread::newplanekeep_thread()
{

}

void newplanekeep_thread::run(){

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

    //设置TCP为法兰中心
    std::vector<double> zero_tcp = {0,0,0,0,0,0};
    TestMainWindow::rtde_control->setTcp(zero_tcp);

    //机械臂初始点（-0.1718,-0.49,0.2,1.606,0,-0）
    std::vector<double> start_pose = {-0.1718,-0.49,0.2,1.606,0,0};
    TestMainWindow::rtde_control->moveL(start_pose,0.1,0.5);
    Eigen::Vector3d  start_position(start_pose[0],start_pose[1],start_pose[2]);

    //向上移动到圆弧的位置
    std::vector<double> start_circle_pose = {-0.1718,-0.49,0.3,1.606,0,0};
    TestMainWindow::rtde_control->moveL(start_circle_pose,0.1,0.5);
    Eigen::Vector3d start_circle_position(start_circle_pose[0],start_circle_pose[1],start_circle_pose[2]);

    //待切平面的三个点
    Eigen::Vector3d plane_pt1(-0.1278,-0.7214,0.031);
    Eigen::Vector3d plane_pt2(-0.20484,-0.6214,0.0714);
    Eigen::Vector3d plane_pt3(-0.20484,-0.7214,0.0714);
    Eigen::Vector4d plane_para = getPlaneBy3Points(plane_pt1,plane_pt2,plane_pt3);

    //自己设一个tcp点
    double plane_tcp[] = {0.029,0.026,0.038,-0.2,-0.2,-0.2};
    Eigen::MatrixXd tcp_trans = ur_tcp_pose2matrix4d(plane_tcp);

    //设定可运动的平面 以基坐标为圆点
    Eigen::Vector3d pt1(-0.1718,-0.49,0.2);
    Eigen::Vector3d pt2(0,-0.49,0);  //该点为绕圆心转动中心点
    Eigen::Vector3d pt3(0,-0.49,0.2);
    Eigen::Vector4d plane_enable = getPlaneBy3Points(pt1,pt2,pt3);

    //添加负载
    std::vector<double> cog = {0,0.1,0};  //质心位置
    TestMainWindow::rtde_control->setPayload(1.1,cog);
    bool circle_move = true;


    //计算开始点与圆心之间的距离
    //qDebug()<<"开始点到圆心的距离：." <<ComputePointToPoint(start_circle_position,pt2);

    double dist_start_center = ComputePointToPoint(start_circle_position,pt2);
    //向上绕圆周运动
//    bool up_circle_move = true;

    //初始化
    double dt = 0.01;
    std::vector<double> new_pose = {0,0,0,0,0,0};

    while (robot_data::start_newplanekeep) {

        if(circle_move == true){
            //机械臂圆弧运动
            robot_data::wrench_ft_frame << robot_data::g_TCP_Force[0],
                                           robot_data::g_TCP_Force[1],
                                           robot_data::g_TCP_Force[2],
                                           0,
                                           0,
                                           0;
        }
        if(circle_move == false){

            robot_data::wrench_ft_frame << 0,
                                           0,
                                           0,
                                           robot_data::g_TCP_Force[3],
                                           robot_data::g_TCP_Force[4],
                                           robot_data::g_TCP_Force[5];
        }

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


         new_pose[0] = robot_data::g_TCP_Pose[0] + cartesian_velocities[0] * dt;
         new_pose[1] = robot_data::g_TCP_Pose[1] + cartesian_velocities[1] * dt;
         new_pose[2] = robot_data::g_TCP_Pose[2] + cartesian_velocities[2] * dt;
         new_pose[3] = robot_data::g_TCP_Pose[3] + cartesian_velocities[3] * dt*30;
         new_pose[4] = robot_data::g_TCP_Pose[4] + cartesian_velocities[4] * dt*30;
         new_pose[5] = robot_data::g_TCP_Pose[5] + cartesian_velocities[5] * dt*30;

         Eigen::Vector3d new_pose_vector3d(new_pose[0],new_pose[1],new_pose[2]);
         //qDebug()<<plane_enable[0]<<plane_enable[1]<<plane_enable[2]<<plane_enable[3];

         //计算空间点到圆上的切点坐标
         Eigen::Vector3d circle_point = getTangentPoint(plane_enable,new_pose_vector3d,pt2,dist_start_center);

         std::vector<double> circle_pose = {circle_point[0],circle_point[1],circle_point[2],new_pose[3],new_pose[4],new_pose[5]};

         //确定机械臂平面的3个点
         Eigen::Vector3d pose_pt1(robot_data::g_TCP_Pose[0],robot_data::g_TCP_Pose[1],robot_data::g_TCP_Pose[2]);
         Eigen::Vector3d pose_pt2(robot_data::g_TCP_Pose[0],robot_data::g_TCP_Pose[1]+0.1,robot_data::g_TCP_Pose[2]);

         //这个是求什么矩阵
         double test_pose[] = {robot_data::g_TCP_Pose[0],robot_data::g_TCP_Pose[1],robot_data::g_TCP_Pose[2],
                               robot_data::g_TCP_Pose[3],robot_data::g_TCP_Pose[4],robot_data::g_TCP_Pose[5]};
         Eigen::MatrixXd test_pose_matrix = ur_tcp_pose2matrix4d(test_pose);
         Eigen::MatrixXd p3_matrix = test_pose_matrix * tcp_trans;

         //qDebug()<<p3_matrix(0,0)<<p3_matrix(0,1)<<p3_matrix(0,2)<<p3_matrix(0,3);
         //qDebug()<<robot_data::g_TCP_Pose[0]<<robot_data::g_TCP_Pose[1]<<robot_data::g_TCP_Pose[2];
         std::vector<double> p3_vector = matrix4d2rv(p3_matrix.data());
         Eigen::Vector3d pose_pt3(p3_vector[0],p3_vector[1],p3_vector[2]);

         //打印待切平面
         qDebug()<<plane_para[0]<<plane_para[1]<<plane_para[2]<<plane_para[3];
         qDebug()<<p3_vector[0]<<p3_vector[1]<<p3_vector[2]<<p3_vector[3]<<p3_vector[4]<<p3_vector[5];

         //计算机械臂所在平面
         Eigen::Vector4d PosePlane_para = getPlaneBy3Points(plane_pt1,plane_pt2,plane_pt3);
         robot_data::new_plane_angle = get_angle(plane_para,PosePlane_para);
         robot_data::pt2plane_angle1 = distanceToPlane(pose_pt1,plane_para);
         robot_data::pt2plane_angle2 = distanceToPlane(pose_pt2,plane_para);
         robot_data::pt2plane_angle3 = distanceToPlane(pose_pt3,plane_para);


//         //判断是否可以向上或向下运动
//         Eigen::Vector3d now_pose{robot_data::g_TCP_Pose[0],robot_data::g_TCP_Pose[1],robot_data::g_TCP_Pose[2]};

//         //如果新的点位到平面的距离小于当前点到平面的距离
//         if(distanceToPlane(now_pose,plane_para)>distanceToPlane(new_pose_vector3d,plane_para))
//         {
//             up_circle_move = false;
//         }


        //qDebug()<<circle_pose[0]<<circle_pose[1]<<circle_pose[2];
         //qDebug()<<"up_circle_move: "<< up_circle_move;
         if(robot_data::start_move==true){
             //qDebug()<<new_pose[0]<<new_pose[1]<<new_pose[2]<<new_pose[3]<<new_pose[4]<<new_pose[5];
             TestMainWindow::rtde_control->servoL(circle_pose, 1, 1, 1.0/500, 0.1, 300);
             if(distanceToPlane(pose_pt1,plane_para)<0.01 && distanceToPlane(pose_pt2,plane_para)<0.01){
                    circle_move = false;
                    TestMainWindow::rtde_control->servoL(new_pose, 1, 1, 1.0/500, 0.1, 300);
                    //TestMainWindow::rtde_control->servoStop();
                    //TestMainWindow::rtde_control->stopScript();
//                    std::vector<int> axes = {0,0,0,0,0,1};
//                    TestMainWindow::rtde_control->freedriveMode(axes);
                    if(distanceToPlane(pose_pt3,plane_para)<0.01){
                        TestMainWindow::rtde_control->endFreedriveMode();

                        //TestMainWindow::rtde_control->servoStop();
                    }
             }
         }
         QThread::msleep(10);
    }

    while(robot_data::start_newplanekeep == false){
        return;
    }

}

Eigen::Vector4d newplanekeep_thread::getPlaneBy3Points(
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


//获得两个平面的夹角
double newplanekeep_thread::get_angle(Eigen::Vector4d n1, Eigen::Vector4d n2){
    double cosθ = abs(n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2]) /
        (sqrt(n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2]) * sqrt(n2[0] * n2[0] + n2[1] * n2[1] + n2[2] * n2[2]));
    double angle = std::acos(cosθ);

    //return angle;
    return angle* 180.0 / 3.1415;
}

// 计算点到平面的距离
double newplanekeep_thread::distanceToPlane(Eigen::Vector3d &P, Eigen::Vector4d &Plane)
{
    double distance = (Plane[0] * P[0] + Plane[1] * P[1] + Plane[2] * P[2] + Plane[3]) /
                      sqrt(pow(Plane[0], 2) + pow(Plane[1], 2) + pow(Plane[2], 2));
    return distance;
}

// tcp转变换矩阵
Eigen::MatrixXd newplanekeep_thread::ur_tcp_pose2matrix4d(double *ur_tcp_pose){
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
std::vector<double> newplanekeep_thread::matrix4d2rv(double *matrix)
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
double newplanekeep_thread::ComputePointToPoint(Eigen::Vector3d &P1, Eigen::Vector3d &P2){
        double dx = P1[0] - P2[0];
        double dy = P1[1] - P2[1];
        double dz = P1[2] - P2[2];

        return sqrt(dx * dx + dy * dy + dz * dz);
}

// 计算空间点到圆上的切点坐标
Eigen::Vector3d newplanekeep_thread::getTangentPoint(const Eigen::Vector4d &plane,
                                const Eigen::Vector3d &pt,
                                const Eigen::Vector3d &center,
                                double radius)
{
    // 求平面的法向量单位化
    double magnitude = sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
    Vector3d normalizedNormal(plane[0] / magnitude, plane[1] / magnitude, plane[2] / magnitude);
    //Eigen::Vector3d normal(plane[0], plane[1], plane[2]);

    // 计算圆心到平面的距离
    //double distanceToPlane = center[0]*normalizedNormal[0]+center[1]*normalizedNormal[1]+center[2]*normalizedNormal[2];

    // 计算任意点到圆心的向量
    double vectorX = pt[0] - center[0];
    double vectorY = pt[1] - center[1];
    double vectorZ = pt[2] - center[2];

//    // 计算从圆心到空间点的向量
//    Eigen::Vector3d vectorToCenter = center - pt;

//    // 将向量投影到法向量上得到在平面上的向量
//    Eigen::Vector3d projectedVector = vectorToCenter - vectorToCenter.dot(normal) * normal;

//    // 将向量单位化并乘以半径r，得到切点向量
//    Eigen::Vector3d tangentVector = projectedVector.normalized() * radius;

    // 计算向量的模长
    double vectorMagnitude = sqrt(vectorX * vectorX + vectorY * vectorY + vectorZ * vectorZ);

    // 计算切点到圆心的距离
    //double tangentDistance = sqrt(vectorMagnitude*vectorMagnitude-radius*radius);

    // 计算切点坐标
    if (abs(vectorMagnitude) < 0.0001)
    {
        return center;
    }
    else
    {
        double scale = radius / vectorMagnitude;
        Vector3d tangentPoint;
        tangentPoint[0] = center[0] + scale * vectorX;
        //tangentPoint[1] = center[1] + scale * vectorY;
        tangentPoint[1] = -0.49;
        tangentPoint[2] = center[2] + scale * vectorZ;
        //tangentPoint=center+tangentVector;
        return tangentPoint;
    }
}

