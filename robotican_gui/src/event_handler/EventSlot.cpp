//
// Created by sub on 14/04/16.
//

#include "EventSlot.h"


EventSlot::EventSlot()
{
}

void EventSlot::initiate(Ui::MainWindow &guiHandle, QApplication &app)
{
    _guiHandle = &guiHandle;
    _app = &app;
}

void EventSlot::setBatPwr(int val)
{
    _guiHandle->battery_pbar->setValue(val);
}

void EventSlot::setMoveState(int state)
{
    ROS_INFO("here---- %i", state);

    switch (state)
        {
            case 0: //canceled
            {
                _guiHandle->move_status_lbl->setStyleSheet("QLabel { background-color : yellow; }");
                _guiHandle->move_status_lbl->setText("Canceled");
                break;
            }
            case 1: //working
            {
                _guiHandle->move_pbar->setVisible(true);
                _guiHandle->move_status_lbl->setStyleSheet("QLabel { background-color : yellow; }");
                _guiHandle->move_status_lbl->setText("Working...");
                _guiHandle->launch_btn->setEnabled(false);
                _guiHandle->preset_btn->setEnabled(false);
                break;
            }
            case 2: //success
            {
                _guiHandle->move_pbar->setVisible(false);
                _guiHandle->move_status_lbl->setText("Success");
                _guiHandle->launch_btn->setEnabled(true);
                _guiHandle->preset_btn->setEnabled(true);
                break;
            }
            case 3: //fail
            {
                _guiHandle->move_pbar->setVisible(false);
                _guiHandle->move_status_lbl->setText("Failed");
                _guiHandle->launch_btn->setEnabled(true);
                _guiHandle->preset_btn->setEnabled(true);
                break;
            }
        }
}

void EventSlot::setLed(long int val, Led* led)
{
    //if no new signal was received by listener
    if (led->getTimeMarker() == val)
    {
        if (calcTimeOut(led->getTimeMarker(), clock()) > TIMEOUT)
            led->turnOff();
    }
    else if (calcTimeOut(led->getTimeMarker(), clock()) != -1)
    {
        led->setTimeMarker(val);
        if (!led->getState())
            led->turnOn();
    }
}

void EventSlot::closeApp()
{
    _app->quit();
}

void EventSlot::execDriveMode()
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(NULL, "Warning", "Are you sure you want to run driving mode?",
                                  QMessageBox::Yes|QMessageBox::No);
    
    if (reply == QMessageBox::Yes)
    {
        setMoveState(1);
        boost::thread worker(&EventSlot::runDriveMode, this);

    } else
    {
        setMoveState(0);
    }

}

bool EventSlot::runDriveMode()
{
     if (_arm.plan())
     {
         //_isSuccess = 2;
         setMoveState(2);
        _arm.move();
     } else
     {
         //_isSuccess = 3;
         setMoveState(3);
     }
}

/************************************************
 * Goal: calculate time interval between times
 * Params: @startTime, @endTime
 * Pre-cond: @endTime > @startTime
 * Post-cond: returns time interval between times
 ***********************************************/
double EventSlot::calcTimeOut(long int startTime, long int endTime)
{
    double timeOut = -1;
    if (endTime > startTime)
    {
        timeOut = (endTime - startTime) / (double) CLOCKS_PER_SEC;
    }
    return timeOut;
}

