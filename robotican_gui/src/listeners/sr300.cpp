
#include "sr300.h"
sr300::sr300()
{
    _signalTime = 0;
}

void sr300::_chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    _signalTime = clock();
}

void sr300::subscribe()
{
    _nHandle.param<std::string>("sr300_topic",_topicName, "sr300/color/image_raw");
    _sub = _nHandle.subscribe(_topicName, 1000, &sr300::_chatterCallback, this);
}

long int sr300::getLastSignal()
{
    return _signalTime;
}