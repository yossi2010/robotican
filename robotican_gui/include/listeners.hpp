#ifndef LISTENERS_H
#define LISTENERS_H

#include <control_msgs/JointTrajectoryControllerState.h>
#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <ric_board/Battery.h>
#include <sensor_msgs/NavSatFix.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/Range.h>

///////////////////LISTENER HEADER///////////////////
class Listener
{
public:
    virtual void subscribe() = 0;
};

///////////////////ARM///////////////////
class Arm : public Listener
{
public:
    Arm() {_signalTime = 0;}
    long int getLastSignal() {return _signalTime;}
    void subscribe()
    {
        _nHandle.param<std::string>("arm_topic",_topicName, "arm_trajectory_controller/state");
        _sub = _nHandle.subscribe(_topicName, 1000, &Arm::_chatterCallback, this);
    }

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg) {_signalTime = clock();}
};

///////////////////BATTERY///////////////////
class Battery : public Listener
{
public:

    Battery()
    {
        _batPower = 0;
        _signalTime = 0;
    };
    int getBatteryPwr() {return _batPower;}
    void subscribe()
    {
        _nHandle.param<std::string>("battery_topic",_topicName, "battery_monitor");
        _sub = _nHandle.subscribe(_topicName, 1000, &Battery::_chatterCallback, this);
    }
    long int getLastSignal() {return _signalTime;}

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    int _batPower;
    long int _signalTime;
    void _chatterCallback(const ric_board::Battery::Ptr& msg)
    {
        if (msg->data > msg->max)
            _batPower = 100;
        else
            _batPower = ( (msg->data - msg->min) / (msg->max - msg->min) ) * 100.0f ;
        _signalTime = clock();
    }
};

///////////////////FRONT CAM///////////////////
class FrontCam : public Listener
{
public:
    FrontCam(){_signalTime = 0;}
    long int getLastSignal() {return _signalTime;}
    void subscribe()
    {
        _nHandle.param<std::string>("front_cam_topic",_topicName, "front_camera/image_raw");
        _sub = _nHandle.subscribe(_topicName, 1000, &FrontCam::_chatterCallback, this);
    }

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::Image::ConstPtr& msg) {_signalTime = clock();}
};

///////////////////GPS///////////////////
class Gps : public Listener
{
public:
    Gps() {_signalTime = 0;}
    long int getLastSignal() {return _signalTime;}
    void subscribe()
    {
        _nHandle.param<std::string>("gps_topic",_topicName, "GPS/fix");
        _sub = _nHandle.subscribe(_topicName, 1000, &Gps::_chatterCallback, this);
    }

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {_signalTime = clock();}
};

///////////////////GRIPPER///////////////////
class Gripper : public Listener
{
public:
    Gripper(){_signalTime = 0;}
    long int getLastSignal() {return _signalTime;}
    void subscribe()
    {
        _nHandle.param<std::string>("gripper_topic",_topicName, "gripper_controller/gripper_cmd/status");
        _sub = _nHandle.subscribe(_topicName, 1000, &Gripper::_chatterCallback, this);
    }

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const actionlib_msgs::GoalStatusArray& msg) {_signalTime = clock();}
};

///////////////////IMU///////////////////
class Imu : public Listener
{
public:
    Imu() {_signalTime = 0;}
    long int getLastSignal() {return _signalTime;}
    void subscribe()
    {
        _nHandle.param<std::string>("imu_topic",_topicName, "IMU/data");
        _sub = _nHandle.subscribe(_topicName, 1000, &Imu::_chatterCallback, this);
    }

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::Imu::ConstPtr& msg) {_signalTime = clock();}
};

///////////////////KINNECT2///////////////////
class Kinect2 : public Listener
{
public:
    Kinect2() {_signalTime = 0;}
    long int getLastSignal(){return _signalTime;}
    void subscribe()
    {
        _nHandle.param<std::string>("kinect2_topic",_topicName, "kinect2/hd/image_depth_rect");
        _sub = _nHandle.subscribe(_topicName, 1000, &Kinect2::_chatterCallback, this);
    }

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::Image::ConstPtr& msg) {_signalTime = clock();}
};

///////////////////LIDAR///////////////////
class Lidar : public Listener
{
public:
    Lidar() {_signalTime = 0;}
    long int getLastSignal() {return _signalTime;}
    void subscribe()
    {
        _nHandle.param<std::string>("lidar_topic",_topicName, "scan");
        _sub = _nHandle.subscribe(_topicName, 1000, &Lidar::_chatterCallback, this);
    }

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg) { _signalTime = clock();}
};

///////////////////ODOM///////////////////
class Odom : public Listener
{
public:
    Odom() {_signalTime = 0;}
    long int getLastSignal() { return _signalTime;}
    void subscribe()
    {
        _nHandle.param<std::string>("odom_topic",_topicName, "mobile_base_controller/odom");
        _sub = _nHandle.subscribe(_topicName, 1000, &Odom::_chatterCallback, this);
    }

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const nav_msgs::Odometry::ConstPtr& msg) {_signalTime = clock();}
};

///////////////////PAN TILT///////////////////

class PanTilt : public Listener
{
public:
    PanTilt() {_signalTime = 0;}
    long int getLastSignal() {return _signalTime;}
    void subscribe()
    {
        _nHandle.param<std::string>("pan_tilt_topic",_topicName, "pan_tilt_trajectory_controller/state");
        _sub = _nHandle.subscribe(_topicName, 1000, &PanTilt::_chatterCallback, this);
    }

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg) {_signalTime = clock();}
};


///////////////////REAR CAM///////////////////
class RearCam : public Listener
{
public:
    RearCam() { _signalTime = 0;}
    long int getLastSignal() {return _signalTime;}
    void subscribe()
    {
        _nHandle.param<std::string>("rear_cam_topic",_topicName, "rear_camera/image_raw");
        _sub = _nHandle.subscribe(_topicName, 1000, &RearCam::_chatterCallback, this);
    }

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::Image::ConstPtr& msg) {_signalTime = clock();}
};

///////////////////SR300///////////////////
class SR300 : public Listener
{
public:
    SR300() {_signalTime = 0;}
    long int getLastSignal() {return _signalTime;}
    void subscribe()
    {
        _nHandle.param<std::string>("sr300_topic",_topicName, "sr300/color/image_raw");
        _sub = _nHandle.subscribe(_topicName, 1000, &SR300::_chatterCallback, this);
    }

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::Image::ConstPtr& msg) { _signalTime = clock();}
};

///////////////////URF LEFT///////////////////
class UrfLeft : public Listener
{
public:
    UrfLeft() {_signalTime = 0;}
    long int getLastSignal() {return _signalTime;}
    void subscribe()
    {
        _nHandle.param<std::string>("urf_left_topic",_topicName, "URF/left");
        _sub = _nHandle.subscribe(_topicName, 1000, &UrfLeft::_chatterCallback, this);
    }

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::Range::ConstPtr& msg) {_signalTime = clock();}
};

///////////////////URF LEFT///////////////////
class UrfRear : public Listener
{
public:
    UrfRear() {_signalTime = 0;}
    long int getLastSignal() {return _signalTime;}
    void subscribe()
    {
        _nHandle.param<std::string>("urf_rear_topic",_topicName, "URF/rear");
        _sub = _nHandle.subscribe(_topicName, 1000, &UrfRear::_chatterCallback, this);
    }

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::Range::ConstPtr& msg) { _signalTime = clock();}
};

///////////////////URF LEFT///////////////////
class UrfRight : public Listener
{
public:
    UrfRight() {_signalTime = 0;}
    long int getLastSignal() {return _signalTime;}
    void subscribe()
    {
        _nHandle.param<std::string>("urf_right_topic",_topicName, "URF/right");
        _sub = _nHandle.subscribe(_topicName, 1000, &UrfRight::_chatterCallback, this);
    }

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::Range::ConstPtr& msg) {_signalTime = clock();}
};

#endif //LISTENERS_H