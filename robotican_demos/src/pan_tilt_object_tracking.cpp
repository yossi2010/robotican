//
// Created by tom on 29/06/16.
//

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
 double pan_position=0,tilt_position=0;

void callback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
    std::vector<std::string> joint_names = joint_state->name;
     pan_position = joint_state->position[find (joint_names.begin(),joint_names.end(), "head_pan_joint") - joint_names.begin()];
      tilt_position = joint_state->position[find (joint_names.begin(),joint_names.end(), "head_tilt_joint") - joint_names.begin()];


}
int main(int argc, char **argv) {
    ros::init(argc, argv, "pan_tilt_api");
    ros::NodeHandle nh;
    ros::Publisher pan_tilt_pub = nh.advertise<std_msgs::Float64MultiArray>("pan_tilt_controller/command", 10);

ros::Subscriber sub = nh.subscribe ("joint_states", 10, callback);

     tf::TransformListener listener;
      tf::StampedTransform transform;
    ros::Rate loopRate(10);
    int count=0;
    while(ros::ok()) {

        try {

            listener.lookupTransform("kinect2_depth_optical_frame", "object_frame", ros::Time(0), transform);
            tf::Vector3 v=transform.getOrigin();
            double pan_err=atan2(v.x(),v.z());
            double tilt_err=atan2(v.y(),v.z());
           //  std::cout <<v.x()<<"    "<<v.y()<<"    "<<v.z()<<std::endl;
             std_msgs::Float64MultiArray multiArray;
             multiArray.data.push_back(pan_position-pan_err); // pan (positive is left)
             multiArray.data.push_back(tilt_position-tilt_err); // tilt (positive is down)
std::cout <<pan_position-pan_err<<"    "<<tilt_position-tilt_err<<std::endl;
if (count >50) pan_tilt_pub.publish(multiArray);
else count++;

    }
    catch (tf::TransformException ex) {
         ROS_ERROR("Pan-Tilt tracking node error: %s",ex.what());
         std_msgs::Float64MultiArray multiArray;
         multiArray.data.push_back(0); // pan (positive is left)
         multiArray.data.push_back(0); // tilt (positive is down)
pan_tilt_pub.publish(multiArray);
    }
       //
        ros::spinOnce();
        loopRate.sleep();
    }


    return 0;


}
