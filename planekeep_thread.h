#ifndef PLANEKEEP_THREAD_H
#define PLANEKEEP_THREAD_H
#include<QThread>
#include "robot_data.h"
#include "testmainwindow.h"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

class planekeep_thread:public QThread
{
    Q_OBJECT
public:
    explicit planekeep_thread(){};

    ///
    /// \brief 三点得平面方程
    /// \param P1
    /// \param P2
    /// \param P3
    /// \return
    ///
    Eigen::Vector4d getPlaneBy3Points(
        const Eigen::Vector3d &P1,
        const Eigen::Vector3d &P2,
        const Eigen::Vector3d &P3);

    bool pointinplane(Eigen::Vector3d &Point, Eigen::Vector4d &Plane);

    double get_angle(Eigen::Vector4d n1, Eigen::Vector4d n2);

    Eigen::MatrixXd ur_tcp_pose2matrix4d(double *ur_tcp_pose);

    std::vector<double> matrix4d2rv(double *matrix);

public slots:
    virtual void run() override;
};

#endif // PLANEKEEP_THREAD_H
