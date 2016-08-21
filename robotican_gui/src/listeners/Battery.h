//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI_Battery_H
#define ROBOTICAN_GUI_Battery_H
#include "Listener.h"
#include <std_msgs/UInt32.h>
#include <ric_board/Battery.h>

class Battery : public Listener
{
public:

    Battery();
    int getBatteryPwr();
    void subscribe();
    long int getLastSignal();

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    int _batPower;
    long int _signalTime;
    void _chatterCallback(const ric_board::Battery::Ptr& msg);

};


#endif //ROBOTICAN_GUI_Battery_H
