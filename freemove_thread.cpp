#include "freemove_thread.h"

freemove_thread::freemove_thread(QObject *parent) : QThread(parent)
{
}

void freemove_thread::freemove_fun(){

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

//    robot_data::D_a_ <<20, 0, 0, 0, 0, 0,
//                        0, 20, 0, 0, 0, 0,
//                        0, 0, 20, 0, 0, 0,
//                        0, 0, 0, 10, 0, 0,
//                        0, 0, 0, 0, 10, 0,
//                        0, 0, 0, 0, 0, 10;

    //添加负载
    std::vector<double> cog = {0,0.1,0};  //质心位置
    TestMainWindow::rtde_control->setPayload(1.1,cog);
    //MainWindow::rtde_control->setTcp(robot_data::Tcp);

    //设置TCP
    std::vector<double> zero_tcp = {0,0,0,0,0,0};
    TestMainWindow::rtde_control->setTcp(zero_tcp);

    while(robot_data::start_freemove==true){

           //读取机械臂的值
           robot_data::wrench_ft_frame << robot_data::g_TCP_Force[0],
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
//                qDebug() << "Admittance generates high arm accelaration!"
//                     << " norm: " << a_acc_norm;

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

            double dt = 0.01;
            std::vector<double> new_pose = {0,0,0,0,0,0};
//            new_pose[0] = robot_data::g_TCP_Pose[0] + cartesian_velocities[0] * dt;
//            new_pose[1] = robot_data::g_TCP_Pose[1] + cartesian_velocities[1] * dt;
//            new_pose[2] = robot_data::g_TCP_Pose[2] + cartesian_velocities[2] * dt;
//            new_pose[3] = robot_data::g_TCP_Pose[3] + cartesian_velocities[3] * dt*10;
//            new_pose[4] = robot_data::g_TCP_Pose[4] + cartesian_velocities[4] * dt*10;
//            new_pose[5] = robot_data::g_TCP_Pose[5] + cartesian_velocities[5] * dt*50;

            if(cartesian_velocities[3]<0){
                cartesian_velocities[3]=0;
            }
            //欧拉角测试
            new_pose[0] = robot_data::g_TCP_Pose[0] ;
            new_pose[1] = robot_data::g_TCP_Pose[1] ;
            new_pose[2] = robot_data::g_TCP_Pose[2] ;
            new_pose[3] = robot_data::g_TCP_Pose[3] + cartesian_velocities[3] * dt*20;
            new_pose[4] = robot_data::g_TCP_Pose[4] ;
            new_pose[5] = robot_data::g_TCP_Pose[5] ;

            if(robot_data::start_move==true){
                //qDebug()<<new_pose[0]<<new_pose[1]<<new_pose[2]<<new_pose[3]<<new_pose[4]<<new_pose[5];
                TestMainWindow::rtde_control->servoL(new_pose, 1, 1, 1.0/500, 0.1, 300);
            }
//            else if(robot_data::start_move ==false){

//            }

        //QThread::msleep(20);
    }

    while (robot_data::start_freemove == false) {
        return;
    }
}

void freemove_thread::run(){
    freemove_fun();
}
