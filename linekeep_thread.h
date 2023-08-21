#ifndef LINEKEEP_THREAD_H
#define LINEKEEP_THREAD_H
#include<QThread>
#include "robot_data.h"
#include "testmainwindow.h"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

class linekeep_thread:public QThread
{
    Q_OBJECT
public:
    explicit linekeep_thread(){};

    ///
    /// \brief 点是否在直线上
    /// \param pt
    /// \param begin
    /// \param end
    /// \return
    ///
    bool PointInLine(const Eigen::Vector3d &pt,
                     const Eigen::Vector3d &begin,
                     const Eigen::Vector3d &end);

    ///
    /// \brief 点到直线的垂足点
    /// \param pt
    /// \param begin
    /// \param end
    /// \return
    ///
    Eigen::Vector3d GetFootOfPerpendicular(const Eigen::Vector3d &pt,
                                           const Eigen::Vector3d &begin,
                                           const Eigen::Vector3d &end);

    ///
    /// \brief 当前点到直线的垂点
    /// \param pt
    /// \param begin
    /// \param end
    /// \return
    ///
    Eigen::Vector3d GetFootOfPerpendicular2(const Eigen::Vector3d &pt,
                                            const Eigen::Vector3d &begin,
                                            const Eigen::Vector3d &end);
signals:


public slots:
    virtual void run() override;
};

#endif // LINEKEEP_THREAD_H
