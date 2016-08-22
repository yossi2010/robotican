//
// Created by sub on 14/04/16.
//

#ifndef ROBOTICAN_GUI_EVENT_SLOT_H
#define ROBOTICAN_GUI_EVENT_SLOT_H
#include "../../include/robotican_gui.h"

#include <sys/wait.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <cstdio>
#include <memory>
#include <ros/ros.h>
#include <QThread>
#include <QFuture>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <QtConcurrentRun>
#include "../gui_components/Led.h"
#include "../event_handler/DriveMode.h"
#include <QMessageBox>


#define TIMEOUT 1.0

class EventSlot : public QThread {
    Q_OBJECT
public:
    EventSlot();
    void initiate(Ui::MainWindow &guiHandle, QApplication &app);

    public Q_SLOTS:
    void setBatPwr(int val);
    void setLed(long int val, Led* led);
    void closeApp();
    void execDriveMode();

private:
    Ui::MainWindow * _guiHandle;
    QApplication * _app;
    ros::NodeHandle _nHandle;
    DriveMode _driveMode;

    double calcTimeOut(long int startTime, long int endTime);
    bool runDriveMode();
};


#endif //ROBOTICAN_GUI_EVENT_SLOT_H
