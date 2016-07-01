//
// Created by tom on 29/06/16.
//

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "pan_tilt_api");
    ros::NodeHandle nodeHandle;
    ros::Publisher pan_tilt_pub = nodeHandle.advertise<std_msgs::Float64MultiArray>("pan_tilt_controller/command", 10);



     tf::TransformListener listener;
      tf::StampedTransform transform;
    ros::Rate loopRate(10);
    while(ros::ok()) {

        try {

            listener.lookupTransform("head_link", "object_frame", ros::Time(0), transform);
            tf::Vector3 v=transform.getOrigin();
            double pan_err=atan2(v.x(),v.z());
             double titlt_err=atan2(v.y(),v.z());
             std::cout <<v.x()<<"    "<<v.y()<<"    "<<v.z()<<"    "<< pan_err*57.29577 <<"    "<<titlt_err*57.29577<<std::endl;
             std_msgs::Float64MultiArray multiArray;
             multiArray.data.push_back(pan_err); // pan
             multiArray.data.push_back(0); // tilt (positive is down)
             pan_tilt_pub.publish(multiArray);
    }
    catch (tf::TransformException ex) {
         ROS_ERROR("Pan-Tilt tracking node error: %s",ex.what());
    }
       //
        ros::spinOnce();
        loopRate.sleep();
    }


    return 0;


}
