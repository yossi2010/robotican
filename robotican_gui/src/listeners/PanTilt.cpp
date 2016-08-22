
#include "PanTilt.h"
PanTilt::PanTilt()
{
    _signalTime = 0;
}

void PanTilt::_chatterCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    _signalTime = clock();
}

void PanTilt::subscribe()
{
    _nHandle.param<std::string>("pan_tilt_topic",_topicName, "pan_tilt_trajectory_controller/state");
    _sub = _nHandle.subscribe(_topicName, 1000, &PanTilt::_chatterCallback, this);
}

long int PanTilt::getLastSignal()
{
    return _signalTime;
}