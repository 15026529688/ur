#ifndef BALLKEEP_THREAD_H
#define BALLKEEP_THREAD_H

#include<QThread>
#include "robot_data.h"
#include "testmainwindow.h"
#include <vtkSmartPointer.h>
#include <vtkSTLReader.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkSphereSource.h>
#include<vtkImplicitPolyDataDistance.h>
#include<vtkPointLocator.h>
#include<vtkCellLocator.h>
#include<vtkGenericCell.h>

class ballkeep_thread: public QThread
{

public:
    explicit ballkeep_thread(){};
    double ComputePointToPoint(Eigen::Vector3d &P1, Eigen::Vector3d &P2);
    void ComputeClosestPoints(vtkPolyData* polyData1, vtkPolyData* polyData2);

public slots:

    virtual void run() override;
};

#endif // BALLKEEP_THREAD_H
