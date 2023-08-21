#include "testmainwindow.h"
#include "ui_testmainwindow.h"
#include <chrono>
#include <QThread>
#include "my_thread.h"
#include <QToolBar>
#include <QtWidgets/QApplication>
#include <QtWidgets\qlabel.h>
#include <QFuture>
#include <QFutureWatcher>


RTDEControlInterface* TestMainWindow::rtde_control=nullptr;
RTDEReceiveInterface* TestMainWindow::rtde_receive=nullptr;
QFuture<void> m_Future;
QFutureWatcher<void> *m_Watcher;

TestMainWindow::TestMainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::TestMainWindow)
{
    ui->setupUi(this);

    //qRegisterMetaType<std::vector<double>>("std::vector<double>");

    resize(1000,800);
    robot_status = false;

    connect(ui->connect_button,&QPushButton::clicked,this,&TestMainWindow::connect_robot);
    connect(ui->getdata_btn,&QPushButton::clicked,this,&TestMainWindow::get_Robotdata);
    connect(ui->freemode_btn,&QPushButton::clicked,this,&TestMainWindow::freemode);
    connect(ui->KeepStraight_btn,&QPushButton::clicked,this,&TestMainWindow::KeepStraight);
    connect(ui->PlaneKeep_btn,&QPushButton::clicked,this,&TestMainWindow::PlaneKeep);
    connect(ui->conekeep_btn,&QPushButton::clicked,this,&TestMainWindow::ConeKeep);
    connect(ui->close_btn,&QPushButton::clicked,this,&TestMainWindow::stop_robot);
    connect(ui->close_btn_2,&QPushButton::clicked,this,&TestMainWindow::stop_robot);
    connect(ui->close_btn_3,&QPushButton::clicked,this,&TestMainWindow::stop_robot);
    connect(ui->halfballkeep_btn,&QPushButton::clicked,this,&TestMainWindow::ball_keep);
    connect(ui->freedrive_btn,&QPushButton::clicked,this,&TestMainWindow::freedrive);
    connect(ui->freedrive_close_btn,&QPushButton::clicked,this,&TestMainWindow::freedrive_close);
    connect(ui->newplane_btn,&QPushButton::clicked,this,&TestMainWindow::new_plane_keep);
    connect(ui->freedrive_btn_2,&QPushButton::clicked,this,&TestMainWindow::freedrive);
    connect(ui->freedrive_close_btn_2,&QPushButton::clicked,this,&TestMainWindow::freedrive_close);

    //设置lineEdit的默认文本
    ui->lineEdit->setText("192.168.0.201");
    ui->lineEdit_2->setText("机器人未连接");
    ui->cone_dist_txt->setText("0.045");
    ui->line_pt1->setText("-0.1157, -0.6287, 0.2253");
    ui->line_pt2->setText("-0.127, -0.348, 0.4794");

}

TestMainWindow::~TestMainWindow()
{
    delete ui;
}

void TestMainWindow::connect_robot(){

    //初始化
    robot_data::start_readdata = false;
    robot_data::start_freemove = false;
    robot_data::start_linekeep = false;
    robot_data::start_conekeep = false;
    robot_data::start_ballkeep = false;
    robot_data::start_planekeep = false;
    robot_data::start_move = false;

    QFuture<void> future=QtConcurrent::run([&]
    {
        QString ip = ui->lineEdit->text();

        rtde_control =  new RTDEControlInterface(ip.toStdString());//"192.168.0.229"
        rtde_receive =  new RTDEReceiveInterface(ip.toStdString());
        robot_status = rtde_control->isConnected();
        //qDebug()<< "robot_status: " <<robot_status;
        if(robot_status == false){
            rtde_control->disconnect();
            rtde_control->reconnect();
        }
    });
    QFutureWatcher<void> watcher;
    watcher.setFuture(future);
    while(watcher.isRunning())
    {
        qApp->processEvents(QEventLoop::AllEvents);
    }
    if(robot_status == true){
        ui->lineEdit_2->setText("机器人已连接");

        robot_data::start_readdata = true;
    }

    //新建线程接受机械臂数据
    QThread *m_thread = new QThread;
    my_thread  *my_work = new my_thread();
    my_work->moveToThread(m_thread);
    m_thread->start();
    my_work->start();

}


void TestMainWindow::get_Robotdata(){

    mTimerId = startTimer(50);

}

//自由拖动
void  TestMainWindow::freemode(){

    new_thread = new QThread;
    freemove_thread *freemove = new freemove_thread();
    freemove->moveToThread(new_thread);
    new_thread->start();

    connect(this,&TestMainWindow::start_freeMove,freemove,&freemove_thread::freemove_fun);
    robot_data::start_freemove = true;
    emit start_freeMove();

}

//直线保持
void  TestMainWindow::KeepStraight(){

    new_thread = new QThread;
    linekeep_thread *line_keep = new linekeep_thread();
    line_keep->moveToThread(new_thread);
    new_thread->start();

    connect(this,&TestMainWindow::start_linekeep,line_keep,&linekeep_thread::run);

    robot_data::start_linekeep = true;
    emit start_linekeep();
}


void  TestMainWindow::PlaneKeep(){
    new_thread = new QThread;
    planekeep_thread *plane_keep = new planekeep_thread();
    plane_keep->moveToThread(new_thread);
    new_thread->start();

    connect(this,&TestMainWindow::start_planekeep,plane_keep,&planekeep_thread::run);

    robot_data::start_planekeep = true;
    emit start_planekeep();

}


void TestMainWindow::ConeKeep(){
    new_thread = new QThread;
    conekeep_thread *cone_keep = new conekeep_thread();
    cone_keep->moveToThread(new_thread);
    new_thread->start();

    QString cone_dist_string = ui->cone_dist_txt->text();
    robot_data::cone_dist = cone_dist_string.toDouble();
    connect(this,&TestMainWindow::start_conekeep,cone_keep,&conekeep_thread::run);

    robot_data::start_conekeep = true;
    emit start_conekeep();
}


void  TestMainWindow::ball_keep(){
    new_thread = new QThread;
    ballkeep_thread *ball_keep = new ballkeep_thread();
    ball_keep->moveToThread(new_thread);
    new_thread->start();

    connect(this,&TestMainWindow::start_ballkeep,ball_keep,&ballkeep_thread::run);
    robot_data::start_ballkeep = true;
    emit start_ballkeep();
}


void TestMainWindow::stop_robot()
{
    robot_data::start_readdata = false;
    robot_data::start_freemove = false;
    robot_data::start_linekeep = false;
    robot_data::start_conekeep = false;
    robot_data::start_ballkeep = false;
    robot_data::start_planekeep = false;
    robot_data::start_move = false;

    //线程关闭
//    new_thread->quit();
//    new_thread->wait();
    new_thread->terminate();
    rtde_control->servoStop();
    rtde_control->stopScript();
    //rtde_control->disconnect();//断开连接
}

void TestMainWindow::timerEvent(QTimerEvent *){

      if(mTimerId<0)
      {
           killTimer(mTimerId);// 杀死定时器
      }

      ui->pose0_txt->setText(QString::number(robot_data::g_TCP_Pose[0]));
      ui->pose1_txt->setText(QString::number(robot_data::g_TCP_Pose[1]));
      ui->pose2_txt->setText(QString::number(robot_data::g_TCP_Pose[2]));
      ui->pose3_txt->setText(QString::number(robot_data::g_TCP_Pose[3]));
      ui->pose4_txt->setText(QString::number(robot_data::g_TCP_Pose[4]));
      ui->pose5_txt->setText(QString::number(robot_data::g_TCP_Pose[5]));

      ui->speed0_txt->setText(QString::number(robot_data::g_TCP_Speed[0]));
      ui->speed1_txt->setText(QString::number(robot_data::g_TCP_Speed[1]));
      ui->speed2_txt->setText(QString::number(robot_data::g_TCP_Speed[2]));
      ui->speed3_txt->setText(QString::number(robot_data::g_TCP_Speed[3]));
      ui->speed4_txt->setText(QString::number(robot_data::g_TCP_Speed[4]));
      ui->speed5_txt->setText(QString::number(robot_data::g_TCP_Speed[5]));

      ui->force0_txt->setText(QString::number(robot_data::g_TCP_Force[0]));
      ui->force1_txt->setText(QString::number(robot_data::g_TCP_Force[1]));
      ui->force2_txt->setText(QString::number(robot_data::g_TCP_Force[2]));
      ui->force3_txt->setText(QString::number(robot_data::g_TCP_Force[3]));
      ui->force4_txt->setText(QString::number(robot_data::g_TCP_Force[4]));
      ui->force5_txt->setText(QString::number(robot_data::g_TCP_Force[5]));

      ui->plane_angle_txt->setText(QString::number(robot_data::plane_angle));
      ui->newplaneangle_txt->setText(QString::number(robot_data::new_plane_angle));

      ui->pt1_dis_txt->setText(QString::number(robot_data::pt2plane_angle1));
      ui->pt2_dis_txt->setText(QString::number(robot_data::pt2plane_angle2));
      ui->pt3_dis_txt->setText(QString::number(robot_data::pt2plane_angle3));

//      QThread::msleep(40);
}

void TestMainWindow::f_GetRobotData(){

}

void TestMainWindow::StartTimer(){

}

void TestMainWindow::futureFinished(){


}


void TestMainWindow::freedrive(){
    std::vector<int> axes = {1,1,1,1,1,1};
    rtde_control->freedriveMode(axes);
}

void TestMainWindow::freedrive_close(){
    rtde_control->endFreedriveMode();
}


void TestMainWindow::new_plane_keep(){
    new_thread = new QThread;
    newplanekeep_thread *newplane_keep = new newplanekeep_thread();
    newplane_keep->moveToThread(new_thread);
    new_thread->start();

    connect(this,&TestMainWindow::start_newplanekeep,newplane_keep,&newplanekeep_thread::run);

    robot_data::start_newplanekeep = true;
    emit start_newplanekeep();

}


//随动校正测试
//
void TestMainWindow::on_initP_btn_clicked()
{

    std::vector<double> zero_tcp = {0,0,0,0,0,0};
    rtde_control->setTcp(zero_tcp);
    //圆心位置
    std::vector<double> start_pose = {0,-0.49,0.2,1.57,0,0};
    rtde_control->moveL(start_pose,0.1,0.5);

//    double ur_tcp_pose[6];
//    for(int i = 0;i<6;i++){
//        ur_tcp_pose[i] = start_pose[i];
//    }
//    Eigen::MatrixXd urMatrix=newplanekeep_thread::ur_tcp_pose2matrix4d(ur_tcp_pose);
//    Eigen::Matrix4d dd;
//    for (int i=0; i<4; i++) {

//        for (int j=0; j<4; j++){
//            dd(i,j)=urMatrix(i,j);
//            //qDebug()<<urMatrix(i,j);//每行元素的分隔符
//          }
//    }
//    Eigen::Matrix3d d=dd.block<3,3>(0,0);
//    Eigen::Quaterniond a(d);
//    Eigen::Vector3d eulerangles = a.toRotationMatrix().eulerAngles(2,1,0);

//    qDebug()<<eulerangles[0]<<eulerangles[1]<<eulerangles[2];

//    double eulerangles2= eulerangles[2]+0.1;

    //向上移动到圆弧的位置
    start_circle_pose = {0,-0.49,0.3,1.57,0,0};
    rtde_control->moveL(start_circle_pose,0.1,0.5);
    Eigen::Vector3d start_circle_position(start_circle_pose[0],start_circle_pose[1],start_circle_pose[2]);

    Eigen::Vector3d pt2(0,-0.49,0.2);  //该点为绕圆心转动中心点
    dist_start_center = newplanekeep_thread::ComputePointToPoint(start_circle_position,pt2); //半径

    //qDebug()<<dist_start_center;

//    double center_x = 0.5;  // 圆心坐标 x
//    double center_y = 0.5;  // 圆心坐标 y
//    double radius = 0.2;    // 圆的半径

}


//基座顺时针测试
void TestMainWindow::on_base_clockwise_btn_clicked()
{
    double angle_step = 0.05;  // 每次步进的角度
    //angle += angle_step;

    //double theta = 1.57;
    if(theta-angle_step>0){
        //for (double angle = 0; angle <= 2 * M_PI; angle += angle_step) {
        double x = dist_start_center * std::cos(theta-angle_step);
        double y = -0.49;
        double z = 0.2 + dist_start_center * std::sin(theta-angle_step);  // 末端高度

        std::cout << "Moving UR arm to (" << x << ", " << y << ", " << z << ")" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 模拟运动时间
        std::vector<double> position = {x,y,z,1.606,0,0};
        //rtde_control->servoL(position,1, 1, 1.0/500, 0.1, 300);
        rtde_control->moveL(position,0.1,0.5);
        //}
        theta -=angle_step;
    }
    else{
        return;
    }
}

//基座逆时针
void TestMainWindow::on_base_counterclockwise_btn_clicked()
{
//    double angle_step = 0.05;  // 每次步进的角度
//    angle -= angle_step;

//    //for (double angle = 0; angle <= 2 * M_PI; angle += angle_step) {
//    double x = -0.1718 - dist_start_center * std::cos(angle);
//    double y = -0.49;
//    double z = 0.3 - dist_start_center * std::sin(angle);  // 末端高度

//    std::cout << "Moving UR arm to (" << x << ", " << y << ", " << z << ")" << std::endl;
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 模拟运动时间
//    std::vector<double> position = {x,y,z,1.606,0,0};
//    //rtde_control->servoL(position,1, 1, 1.0/500, 0.1, 300);
//    rtde_control->moveL(position,0.1,0.5);


    double angle_step = 0.05;  // 每次步进的角度
    //angle += angle_step;

    //double theta = 1.57;
    if(theta+angle_step<3.14){
        double x = dist_start_center * std::cos(theta+angle_step);
        double y = -0.49;
        double z = 0.2 + dist_start_center * std::sin(theta+angle_step);  // 末端高度

        std::cout << "Moving UR arm to (" << x << ", " << y << ", " << z << ")" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 模拟运动时间
        std::vector<double> position = {x,y,z,1.606,0,0};
        rtde_control->moveL(position,0.1,0.5);
        theta +=angle_step;
    }
    else{
        return;
    }
}

//Z轴顺时针
void TestMainWindow::on_base_counterclockwise_btn_2_clicked()
{

    double angle_step = 0.05;  // 每次步进的角度
    std::vector<double> joints =rtde_receive->getActualQ();
    for(int i=0;i<6;i++){
        Z_position[i] = joints[i];
    }

    double z_angle = joints[5]-angle_step;
    Z_position[5]= z_angle;
    rtde_control->moveJ(Z_position, 1.05, 1.4);
    qDebug()<<Z_position[5];

}


//Z轴逆时针
void TestMainWindow::on_Z_clockwise_btn_clicked()
{
    double angle_step = 0.05;  // 每次步进的角度
    std::vector<double> joints =rtde_receive->getActualQ();
    for(int i=0;i<6;i++){
        Z_position[i] = joints[i];
    }

    double z_angle = joints[5]+angle_step;
    Z_position[5]= z_angle;
    rtde_control->moveJ(Z_position, 1.05, 1.4);
    qDebug()<<Z_position[5];
}

//x轴顺时针
void TestMainWindow::on_Rx_clockwise_btn_clicked()
{
    std::vector<double> zero_tcp = {0,0,0,0,0,0};
    rtde_control->setTcp(zero_tcp);
    double angle_step = 0.05;  // 每次步进的角度
    std::vector<double> X_position = robot_data::g_TCP_Pose;
    double x_angle = robot_data::g_TCP_Pose[4]+angle_step;
    X_position[4]= x_angle;
    rtde_control->moveL(X_position,0.1,0.5);

}

//x轴逆时针
void TestMainWindow::on_Rx_counterclockwise_btn_clicked()
{
    std::vector<double> zero_tcp = {0,0,0,0,0,0};
    rtde_control->setTcp(zero_tcp);
    double angle_step = 0.05;  // 每次步进的角度
    std::vector<double> X_position = robot_data::g_TCP_Pose;
    double x_angle = robot_data::g_TCP_Pose[4]-angle_step;
    X_position[4]= x_angle;
    rtde_control->moveL(X_position,0.1,0.5);

}

void TestMainWindow::on_pushButton_clicked()
{
    qDebug()<<QDateTime::currentDateTime();

    std::vector<double> init_q = robot_data::g_TCP_Pose;

    // Target in the robot base
    std::vector<double> new_q = init_q;

    for(int i =0;i<5;i++){
        new_q[2] += 0.001;
        rtde_control->moveL(new_q);
        qDebug()<<QDateTime::currentDateTime();
    }

}
