//
// Created by tom on 29/06/16.
//

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

template <typename T> int sgn(T val);
void callback(const sensor_msgs::JointState::ConstPtr& joint_state);

double pan_position=0,tilt_position=0;
bool have_pan=false,have_tilt=false;

void callback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
    // std::vector<std::string> joint_names = joint_state->name;
    for (int i=0;i<joint_state->name.size();i++)
    {
        if (joint_state->name[i] == "head_pan_joint")
        {
            pan_position = joint_state->position[i];
            if (!have_pan) have_pan=true;
        }
        else if (joint_state->name[i] == "head_tilt_joint")
        {
            tilt_position = joint_state->position[i];
            if (!have_tilt) have_tilt=true;
        }
    }

    // tilt_position = joint_state->position[find (joint_names.begin(),joint_names.end(), "head_tilt_joint") - joint_names.begin()];


}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pan_tilt_api");
    ros::NodeHandle nh;
    ros::Publisher pan_tilt_pub = nh.advertise<std_msgs::Float64MultiArray>("pan_tilt_controller/command", 10);

    ros::Subscriber sub = nh.subscribe ("joint_states", 10, callback);
    double angle_delta;
    std::string object_frame,depth_camera_frame;
    nh.param<double>("angle_delta", angle_delta, 0.003);
    nh.param<std::string>("object_frame", object_frame, "object_frame");
    nh.param<std::string>("depth_camera_frame", depth_camera_frame, "kinect2_depth_optical_frame");
    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Rate loopRate(30);
ROS_INFO("Waiting for tranformations...");
    listener.waitForTransform(depth_camera_frame, object_frame,ros::Time::now(), ros::Duration(10.0));
ROS_INFO("Ready to track!");
    while(ros::ok()) {
        if ((have_tilt)&&(have_pan)) {
            try {

                listener.lookupTransform(depth_camera_frame, object_frame, ros::Time(0), transform);
                tf::Vector3 v=transform.getOrigin();
                double pan_err=atan2(v.x(),v.z());
                double tilt_err=atan2(v.y(),v.z());
                std_msgs::Float64MultiArray multiArray;
                double pan_cmd=pan_position-sgn(pan_err)*angle_delta;
                double tilt_cmd=tilt_position+sgn(tilt_err)*angle_delta;
                multiArray.data.push_back(pan_cmd); // pan (positive is left)
                multiArray.data.push_back(tilt_cmd); // tilt (positive is down)
                pan_tilt_pub.publish(multiArray);


            }
            catch (tf::TransformException ex) {
                ROS_ERROR("Pan-Tilt tracking node error: %s",ex.what());
                std_msgs::Float64MultiArray multiArray;
                multiArray.data.push_back(pan_position); // pan (positive is left)
                multiArray.data.push_back(tilt_position); // tilt (positive is down)
                if ((have_tilt)&&(have_pan)) pan_tilt_pub.publish(multiArray);
            }
        }
        //
        ros::spinOnce();
        loopRate.sleep();
    }


    return 0;


}
