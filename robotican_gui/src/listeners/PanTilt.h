//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI_PanTiltListener_H
#define ROBOTICAN_GUI_PanTiltListener_H
#include "listeners/Listener.h"
#include <control_msgs/JointTrajectoryControllerState.h>

class PanTilt : public Listener
{
public:
    PanTilt();
    long int getLastSignal();
    void subscribe();

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);
};


#endif //ROBOTICAN_GUI_PanTiltListener_H
