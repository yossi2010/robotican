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
    
    if (reply == QMessageBox::Yes) {

        boost::thread worker(&EventSlot::runDriveMode, this);
    } else {
    }


}

bool EventSlot::runDriveMode()
{
     _driveMode.moveArm();
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