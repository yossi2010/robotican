#ifndef GUI_MANAGER_H
#define GUI_MANAGER_H

#include <sstream>
#include <vector>
#include <tinyxml.h>
#include <ros/package.h>
#include "robotican_gui.h"
#include "event_signal.h"
#include "event_slot.h"
#include "led.h"

#include "listeners.hpp"

#define LOOP_RATE 0.001

class GUImanager {
private:

    EventSignal _eventSignal;
    EventSlot _eventSlot;
    QMainWindow * _widget;
    Ui::MainWindow * _win;
    QApplication * _app;
    ros::NodeHandle _nh;
    ros::Timer _timer;

    Battery _batListener;
    Arm _armListener;
    PanTilt _panTiltListenere;
    Odom _odomListener;
    Gripper _gripperListener;
    Imu _imuListener;
    Lidar _lidarListener;
    Gps _gpsListener;
    FrontCam _frontCamListener;
    RearCam _rearCamListener;
    UrfLeft _urfLeftListener;
    UrfRear _urfRearListener;
    UrfRight _urfRightListener;
    Kinect2 _kinect2Listener;
    SR300 _sr300;

    Led _armLed;
    Led _panTiltLed;
    Led _odomLed;
    Led _gripperLed;
    Led _imuLed;
    Led _lidarLed;
    Led _gpsLed;
    Led _frontCamLed;
    Led _rearCamLed;
    Led _urfLeftLed;
    Led _urfRearLed;
    Led _urfRightLed;
    Led _kinect2Led;
    Led _sr300Led;
    Led _batteryLed;

    void _loopEvents(const ros::TimerEvent& timerEvent);
    void _connectEvents();
    void _subscribeListeners();
    void _initiateLeds();
    void _initiateLbls();

public:
    GUImanager(QMainWindow &widget, Ui::MainWindow &win, QApplication &app);
    void startGUI();
};


#endif //GUI_MANAGER_H
