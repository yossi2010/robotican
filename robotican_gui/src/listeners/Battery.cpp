//
// Created by sub on 16/04/16.
//

//
// Created by sub on 13/04/16.
//
#include "Battery.h"

Battery::Battery()
{
    _batPower = 0;
    _signalTime = 0;
}

void Battery::_chatterCallback(const ric_board::Battery::Ptr& msg)
{
    _batPower = ( msg->data / (msg->max - msg->min) ) * 100 ;
    _signalTime = clock();
}

int Battery::getBatteryPwr()
{
    return _batPower;
}

void Battery::subscribe()
{
    _nHandle.param<std::string>("battery_monitor",_topicName, "battery_topic");
    _sub = _nHandle.subscribe(_topicName, 1000, &Battery::_chatterCallback, this);
}

long int Battery::getLastSignal()
{
    return _signalTime;
}