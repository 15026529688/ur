#ifndef FREEMOVE_THREAD_H
#define FREEMOVE_THREAD_H

#include <QObject>
#include<QThread>
#include "robot_data.h"
#include "testmainwindow.h"

class freemove_thread : public QThread
{
    Q_OBJECT
public:
    explicit freemove_thread(QObject *parent = nullptr);



signals:


public slots:
    void freemove_fun();
    virtual void run() override;
};

#endif // FREEMOVE_THREAD_H
