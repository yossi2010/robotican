//
// Created by sub on 14/04/16.
//

#include "EventSlot.h"


EventSlot::EventSlot() {}

void EventSlot::initiate(Ui::MainWindow &guiHandle, QApplication &app)
{
    _guiHandle = &guiHandle;
    _app = &app;
}

void EventSlot::setBatPwr(int val)
{
    _guiHandle->battery_pbar->setValue(val);
}

void EventSlot::setMoveState(Status state)
{
    switch (state)
        {
            case CANCELED:
            {
                _guiHandle->move_status_lbl->setStyleSheet("QLabel { background-color : yellow; color : black }");
                _guiHandle->move_status_lbl->setText("Canceled");
                break;
            }
            case WORKING:
            {
                _guiHandle->move_pbar->setVisible(true);
                _guiHandle->cmbox_preset->setEnabled(false);
                _guiHandle->move_status_lbl->setStyleSheet("QLabel { background-color : yellow; color : black }");
                _guiHandle->move_status_lbl->setText("Planning...");
                _guiHandle->drive_btn->setEnabled(false);
                _guiHandle->preset_btn->setEnabled(false);
                break;
            }
            case SUCCESS:
            {
                _guiHandle->cmbox_preset->setEnabled(true);
                _guiHandle->move_pbar->setVisible(false);
                _guiHandle->move_status_lbl->setStyleSheet("QLabel { background-color : green; color : white }");
                _guiHandle->move_status_lbl->setText("Success");
                _guiHandle->drive_btn->setEnabled(true);
                _guiHandle->preset_btn->setEnabled(true);
                break;
            }
            case FAIL:
            {
                _guiHandle->cmbox_preset->setEnabled(true);
                _guiHandle->move_pbar->setVisible(false);
                _guiHandle->move_status_lbl->setStyleSheet("QLabel { background-color : red; color : white }");
                _guiHandle->move_status_lbl->setText("Failed");
                _guiHandle->drive_btn->setEnabled(true);
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

void EventSlot::moveArm()
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(NULL, "Warning", QString::fromStdString(_userMsg),
                                  QMessageBox::Yes|QMessageBox::No);

    if (reply == QMessageBox::Yes)
    {
        setMoveState(WORKING);
        boost::thread worker(&EventSlot::execMove, this);

    } else
        setMoveState(CANCELED);
}
void EventSlot::moveArmToDrive()
{
    _targetName = "driving";
    _userMsg = "Are you sure you want to run driving mode?";
    moveArm();
}

void EventSlot::moveArmToPreset()
{
    QString selection = _guiHandle->cmbox_preset->currentText();
    _targetName = selection.toStdString();
    _userMsg = "Are you sure you want to move the arm?";
    moveArm();
}

bool EventSlot::execMove()
{
     if (_arm.plan(_targetName))
     {
         setMoveState(SUCCESS);
        _arm.move();
     } else
         setMoveState(FAIL);
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
        timeOut = (endTime - startTime) / (double) CLOCKS_PER_SEC;
    return timeOut;
}

