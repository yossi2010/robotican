//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI_sr300Listener_H
#define ROBOTICAN_GUI_sr300Listener_H
#include "listeners/Listener.h"
#include <sensor_msgs/Image.h>

class sr300 : public Listener
{
public:
    sr300();
    long int getLastSignal();
    void subscribe();

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::Image::ConstPtr& msg);
};

#endif //ROBOTICAN_GUI_sr300Listener_H
