#include "ballkeep_thread.h"
#include<vtkCleanPolyData.h>
#include<vtkDistancePolyDataFilter.h>
#include <vtkPointData.h>
#include <vtkLine.h>
void ballkeep_thread::run(){

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

    robot_data::D_a_ <<20, 0, 0, 0, 0, 0,
                        0, 20, 0, 0, 0, 0,
                        0, 0, 20, 0, 0, 0,
                        0, 0, 0, 10, 0, 0,
                        0, 0, 0, 0, 10, 0,
                        0, 0, 0, 0, 0, 10;

//    robot_data::M_a_ <<10, 0, 0, 0, 0, 0,
//                        0, 10, 0, 0, 0, 0,
//                        0, 0, 10, 0, 0, 0,
//                        0, 0, 0, 5, 0, 0,
//                        0, 0, 0, 0, 5, 0,
//                        0, 0, 0, 0, 0, 5;


//    robot_data::D_a_ <<60, 0, 0, 0, 0, 0,
//                        0, 60, 0, 0, 0, 0,
//                        0, 0, 60, 0, 0, 0,
//                        0, 0, 0, 20, 0, 0,
//                        0, 0, 0, 0, 20, 0,
//                        0, 0, 0, 0, 0,20;

    //添加负载
    std::vector<double> cog = {0,0.02,0};  //质心位置
    TestMainWindow::rtde_control->setPayload(0.1,cog);

    TestMainWindow::rtde_control->setTcp(robot_data::Tcp);

    //测试位置
    std::vector<double> test_pose={-0.093,-0.696,0.063,1.16,0.568,0.246};

    TestMainWindow::rtde_control->moveL(test_pose);

//    //开始位置
//    std::vector<double> start_pose={-0.093,-0.696,0.11,1.16,0.568,0.246};
//    MainWindow::rtde_control->moveL(start_pose);

    //读取STL文件
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName("D:/QTproject/ur_control/half_Sphere_100.stl");
    reader->Update();

    vtkSmartPointer<vtkTransform> translation = vtkSmartPointer<vtkTransform>::New();
    translation->Translate(-93,-696,63); //设置平移坐标
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputData(reader->GetOutput());//数据输入
    transformFilter->SetTransform(translation);
    transformFilter->Update();

    // Create the tree
      auto cellLocator = vtkSmartPointer<vtkCellLocator>::New();
      cellLocator->SetDataSet(transformFilter->GetOutput());
      cellLocator->BuildLocator();

      auto assistCell = vtkSmartPointer<vtkGenericCell>::New();
      double closestPoint[3];//the coordinates of the closest point will be returned here
      double closestPointDist2; //the squared distance to the closest point will be returned here
      vtkIdType cellId; //the cell id of the cell containing the closest point will be returned here
      int subId;


    vtkSmartPointer<vtkPoints> tcp_point = vtkSmartPointer<vtkPoints>::New();
     vtkSmartPointer<vtkSphereSource> sphereSource1 = vtkSmartPointer<vtkSphereSource>::New();
     //vtkSmartPointer<vtkCleanPolyData> clean1 = vtkSmartPointer<vtkCleanPolyData>::New();
     vtkSmartPointer<vtkDistancePolyDataFilter> distanceFilter = vtkSmartPointer<vtkDistancePolyDataFilter>::New();

    vtkSmartPointer<vtkPolyData> pointdata = vtkSmartPointer<vtkPolyData>::New();

    tcp_point->InsertPoint(0,0, 0, 0);
    pointdata->SetPoints(tcp_point);

     while(robot_data::start_ballkeep){

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
            robot_data::arm_desired_accelaration.segment(0, 3) *=
                    (robot_data::arm_max_acc_ / a_acc_norm);
        }

        robot_data::arm_desired_twist_ += robot_data::arm_desired_accelaration * 0.1;

        std::vector<double> cartesian_velocities{robot_data::arm_desired_twist_[0],
                                                 robot_data::arm_desired_twist_[1],
                                                 robot_data::arm_desired_twist_[2],
                                                 robot_data::arm_desired_twist_[3],
                                                 robot_data::arm_desired_twist_[4],
                                                 robot_data::arm_desired_twist_[5]};
        //假设球心位置
//        robot_data::ball_center.Zero();
//        robot_data::ball_center={-0.093,-0.696,0.063};
        double dt = 0.01;
        std::vector<double> new_pose = {0,0,0,0,0,0};
        new_pose[0] = robot_data::g_TCP_Pose[0] + cartesian_velocities[0] * dt;
        new_pose[1] = robot_data::g_TCP_Pose[1] + cartesian_velocities[1] * dt;
        new_pose[2] = robot_data::g_TCP_Pose[2] + cartesian_velocities[2] * dt;
        new_pose[3] = robot_data::g_TCP_Pose[3] + cartesian_velocities[3] * dt;
        new_pose[4] = robot_data::g_TCP_Pose[4] + cartesian_velocities[4] * dt;
        new_pose[5] = robot_data::g_TCP_Pose[5] + cartesian_velocities[5] * dt;

        //tcp的位置矩阵
        Eigen::Vector3d P(new_pose[0],new_pose[1],new_pose[2] );

        double testPoint[3] = {P[0]*1000, P[1]*1000, P[2]*1000};

        cellLocator->FindClosestPoint(testPoint, closestPoint, assistCell, cellId, subId, closestPointDist2);

        qDebug()<<sqrt(closestPointDist2);

        if(robot_data::start_move==true){
            TestMainWindow::rtde_control->servoL(new_pose, 1, 1, 1.0/500, 0.1, 300);

            if(sqrt(closestPointDist2)>3){
                TestMainWindow::rtde_control->servoL(new_pose, 1, 1, 1.0/500, 0.1, 300);
            }
            else if(sqrt(closestPointDist2)<3&&sqrt(closestPointDist2)>2.5){
                TestMainWindow::rtde_control->servoL(new_pose, 1, 1, 1.0/500, 0.1, 300);
                robot_data::ball_queue.push(robot_data::g_TCP_Pose);
                if(robot_data::ball_queue.size()>10){
                    robot_data::ball_queue.pop();
                }
            }
            else if (sqrt(closestPointDist2)<2) {
                //MainWindow::rtde_control->speedStop();
                TestMainWindow::rtde_control->servoL(robot_data::ball_queue.back(),1,1,1.0/500, 0.1, 300);
            }
        }


        else if(robot_data::start_move ==false){

        }
        QThread::msleep(10);
    }
    while (robot_data::start_ballkeep == false) {
        return;
    }
}

// 点到点的距离
double ballkeep_thread::ComputePointToPoint(Eigen::Vector3d &P1, Eigen::Vector3d &P2){
        double dx = P1[0] - P2[0];
        double dy = P1[1] - P2[1];
        double dz = P1[2] - P2[2];

        return sqrt(dx * dx + dy * dy + dz * dz);
}

void ballkeep_thread::ComputeClosestPoints(vtkPolyData* polyData1, vtkPolyData* polyData2)
{
    // 创建 vtkImplicitPolyDataDistance 对象
    vtkSmartPointer<vtkImplicitPolyDataDistance> distance = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
    distance->SetInput(polyData2);

    // 创建 vtkPointLocator 对象
    vtkSmartPointer<vtkPointLocator> pointLocator = vtkSmartPointer<vtkPointLocator>::New();
    pointLocator->SetDataSet(polyData1);
    pointLocator->BuildLocator();

    // 遍历 polyData1 的所有点，计算最近距离
    double closestPoint[3];
    double closestDistance = VTK_DOUBLE_MAX;
    vtkIdType closestPointId = -1;
    for (vtkIdType pointId = 0; pointId < polyData1->GetNumberOfPoints(); ++pointId)
    {
        double point[3];
        polyData1->GetPoint(pointId, point);

        // 通过 vtkImplicitPolyDataDistance 计算最近距离
        double distanceValue = distance->EvaluateFunction(point);
        if (distanceValue < closestDistance)
        {
            closestDistance = distanceValue;
            closestPointId = pointLocator->FindClosestPoint(point);
            polyData2->GetPoint(closestPointId, closestPoint);
        }
    }

    // 输出最近距离和对应的点
    std::cout << "Closest distance: " << closestDistance << std::endl;
//    std::cout << "Closest point on polyData1: (" << closestPoint[0] << ", " << closestPoint[1] << ", " << closestPoint[2] << ")" << std::endl;
//    std::cout << "Closest point on polyData2: (" << closestPoint[0] << ", " << closestPoint[1] << ", " << closestPoint[2] << ")" << std::endl;

}

