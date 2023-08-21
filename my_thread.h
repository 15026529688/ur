#ifndef MY_THREAD_H
#define MY_THREAD_H

#include<QObject>
#include<QThread>
#include"robot_data.h"

class my_thread:public QThread
{
    Q_OBJECT
public:
    explicit  my_thread(QObject *parent =nullptr);

public slots:
    void fun();
virtual void run() override;


};

#endif // MY_THREAD_H
