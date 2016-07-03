//
// Created by tom on 29/06/16.
//

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>



int main(int argc, char **argv) {
    ros::init(argc, argv, "pan_tilt_api");
    ros::NodeHandle nodeHandle;
    ros::Publisher pan_tilt_pub = nodeHandle.advertise<std_msgs::Float64MultiArray>("pan_tilt_controller/command", 10);

    std_msgs::Float64MultiArray multiArray;
    multiArray.data.push_back(0.2); // pan
    multiArray.data.push_back(0.3); // tilt (positive is down)
    multiArray.data.push_back(0.0); // pan
    multiArray.data.push_back(0.0); // tilt
    ros::Rate loopRate(1);
    while(ros::ok()) {
        pan_tilt_pub.publish(multiArray);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;


}
