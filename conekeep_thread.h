#ifndef CONEKEEP_THREAD_H
#define CONEKEEP_THREAD_H
#include <QObject>
#include<QThread>
#include "robot_data.h"
#include "testmainwindow.h"
#include<queue>

class conekeep_thread : public QThread
{
    Q_OBJECT
public:
    explicit conekeep_thread(){};

    Eigen::MatrixXd ur_tcp_pose2matrix4d(double *ur_tcp_pose);

    std::vector<double> matrix4d2rv(double *matrix);

    double ComputePointToPoint(Eigen::Vector3d &P1, Eigen::Vector3d &P2);

signals:


public slots:

    virtual void run() override;
};

#endif // CONEKEEP_THREAD_H
