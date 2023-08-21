#include "robot_data.h"

std::vector<double> robot_data::g_TCP_Pose = {0,0,0,0,0,0};
std::vector<double> robot_data::g_TCP_Speed = {0,0,0,0,0,0};
std::vector<double> robot_data::g_TCP_Force = {0,0,0,0,0,0};

Matrix6d robot_data::M_a_ ;
//(6, 0, 0, 0, 0, 0,
//                            0, 6, 0, 0, 0, 0,
//                            0, 0, 6, 0, 0, 0,
//                            0, 0, 0, 2, 0, 0,
//                            0, 0, 0, 0, 2, 0,
//                            0, 0, 0, 0, 0, 2);

Matrix6d robot_data::D_a_ ;
//(40, 0, 0, 0, 0, 0,
//                            0, 40, 0, 0, 0, 0,
//                            0, 0, 40, 0, 0, 0,
//                            0, 0, 0, 15, 0, 0,
//                            0, 0, 0, 0, 15, 0,
//                            0, 0, 0, 0, 0, 15);

double robot_data::arm_max_acc_ = 0.3;

Vector6d robot_data::arm_desired_accelaration;

Vector6d robot_data::arm_desired_twist_;

Vector6d robot_data::wrench_external_;

Vector6d robot_data::wrench_ft_frame;

bool robot_data::start_freemove = false;

bool robot_data::start_linekeep = false;

//先设定这两个值
Vector3d robot_data::line_point1(-0.1157, -0.6287, 0.2253);
Vector3d robot_data::line_point2(-0.127, -0.348, 0.4794);

bool robot_data::start_planekeep = false;

bool robot_data::start_conekeep = false;

std::vector<double> robot_data::cone_center = {0,0,0,0,0,0};

std::vector<double> robot_data::Tcp = {-0.0019, 0.03437, 0.17389, -1.8538, -0.291, -0.2192};

double robot_data::double_tcp[6] = {-0.0019, 0.03437, 0.17389, -1.8538, -0.291, -0.2192};

std::vector<double> robot_data::frange_fromTcp={0,0,0,0,0,0};

std::queue<std::vector<double>> robot_data::pose_queue;

std::queue<std::vector<double>> robot_data::ball_queue;

bool robot_data::start_ballkeep= false;

Vector3d robot_data::ball_center;

bool robot_data::start_readdata = false;

double robot_data::plane_angle = 0;

bool robot_data::start_move = false;

double robot_data::cone_dist = 0;

bool robot_data::start_find_plane= false;

bool robot_data::start_newplanekeep = false;

double robot_data::new_plane_angle = 0;

double robot_data::pt2plane_angle1 = 0;
double robot_data::pt2plane_angle2 = 0;
double robot_data::pt2plane_angle3 = 0;
