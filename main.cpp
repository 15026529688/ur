#include "testmainwindow.h"

#include <QApplication>
#include <QtWidgets/QApplication>
#include <QtWidgets\qlabel.h>


int main(int argc, char *argv[])
{
    #if (QT_VERSION >= QT_VERSION_CHECK(5, 6, 0))
        QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    #endif

    QApplication a(argc, argv);
    TestMainWindow w;
    w.show();
    return a.exec();
}

//#include <iostream>
//#include <Eigen/Dense>

//int main() {
//    // 定义圆所在平面的法向量和圆心坐标
//    double Nx = 1.0;
//    double Ny = 2.0;
//    double Nz = 3.0;
//    double Cx = 4.0;
//    double Cy = 5.0;
//    double Cz = 6.0;

//    // 定义空间点坐标
//    double Px = 7.0;
//    double Py = 8.0;
//    double Pz = 9.0;

//    // 定义圆的半径
//    double r = 1.0;

//    // 创建Eigen向量表示法向量、圆心和空间点
//    Eigen::Vector3d normal(Nx, Ny, Nz);
//    Eigen::Vector3d center(Cx, Cy, Cz);
//    Eigen::Vector3d point(Px, Py, Pz);

//    // 计算从圆心到空间点的向量
//    Eigen::Vector3d vectorToCenter = center - point;

//    // 计算单位法向量
//    Eigen::Vector3d unitNormal = normal.normalized();

//    // 计算空间点到平面圆上的切点向量
//    Eigen::Vector3d tangentVector = vectorToCenter - vectorToCenter.dot(unitNormal) * unitNormal;

//    // 计算切点坐标
//    Eigen::Vector3d tangentPoint = point + tangentVector.normalized() * r;

//    // 输出切点坐标
//    std::cout << "切点坐标为：" << tangentPoint << std::endl;

//    return 0;
//}
