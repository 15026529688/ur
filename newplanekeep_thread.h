#ifndef NEWPLANEKEEP_THREAD_H
#define NEWPLANEKEEP_THREAD_H

#include <QObject>
#include<QThread>
#include "robot_data.h"
#include "testmainwindow.h"

class newplanekeep_thread: public QThread
{
public:
    explicit newplanekeep_thread();
    Eigen::Vector4d getPlaneBy3Points(
            const Eigen::Vector3d &P1,
            const Eigen::Vector3d &P2,
            const Eigen::Vector3d &P3);

    double get_angle(Eigen::Vector4d n1, Eigen::Vector4d n2);
    double distanceToPlane(Eigen::Vector3d &P, Eigen::Vector4d &Plane);

    static Eigen::MatrixXd ur_tcp_pose2matrix4d(double *ur_tcp_pose);

    std::vector<double> matrix4d2rv(double *matrix);

    static double ComputePointToPoint(Eigen::Vector3d &P1, Eigen::Vector3d &P2);

    // 计算空间点到圆上的切点坐标
    Eigen::Vector3d getTangentPoint(const Eigen::Vector4d &plane,
                                    const Eigen::Vector3d &pt,
                                    const Eigen::Vector3d &center,
                                    double radius);


public slots:
    virtual void run() override;
};

#endif // NEWPLANEKEEP_THREAD_H
