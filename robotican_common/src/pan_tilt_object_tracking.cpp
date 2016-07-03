

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

template <typename T> int sgn(T val);
void callback(const sensor_msgs::JointState::ConstPtr& joint_state);
void reset();

double pan_position=0,tilt_position=0;
bool have_pan=false,have_tilt=false;
ros::Publisher pan_tilt_pub;


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

void reset() {

    std_msgs::Float64MultiArray multiArray;
    multiArray.data.push_back(0); // pan (positive is left)
    multiArray.data.push_back(0); // tilt (positive is down)
    pan_tilt_pub.publish(multiArray);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pan_tilt_api");
    ros::NodeHandle nh;
    pan_tilt_pub = nh.advertise<std_msgs::Float64MultiArray>("pan_tilt_controller/command", 10);
    ros::Subscriber sub = nh.subscribe ("joint_states", 10, callback);
    double max_rate,kp_pan,kp_tilt,kd_pan,kd_tilt,pre_pan_err=0,pre_tilt_err=0,target_tol;
    std::string object_frame,depth_camera_frame;
    double no_object_timeout;
    double loop_hz;
    nh.param<double>("max_rate", max_rate, 0.3); //rad/s
    nh.param<std::string>("object_frame", object_frame, "object_frame");
    nh.param<double>("kp_pan", kp_pan, 4.0);
    nh.param<double>("kp_tilt", kp_tilt, 3.0);
    nh.param<double>("kd_pan", kd_pan, 0.01);
    nh.param<double>("kd_tilt", kd_tilt, 0.01);
    nh.param<double>("loop_hz", loop_hz, 30);
    nh.param<double>("target_tol", target_tol,1.0*M_PI/180.0);
     nh.param<double>("no_object_timeout", no_object_timeout,5.0);
    nh.param<std::string>("depth_camera_frame", depth_camera_frame, "kinect2_depth_optical_frame");

    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Duration dt;
    ros::Rate loopRate(loop_hz);
    ros::Time pre_t=ros::Time::now();


    while (!listener.waitForTransform(depth_camera_frame, object_frame,ros::Time::now(), ros::Duration(1.0))) {
        ROS_INFO("Waiting for tranformations...");
    }
    ROS_INFO("Ready to track!");
    bool first=true;
    while(ros::ok()) {
        if ((have_tilt)&&(have_pan)) {
            try {

                listener.lookupTransform(depth_camera_frame, object_frame, ros::Time(0), transform);

                ros::Duration d=ros::Time::now()-transform.stamp_;
                if (d.toSec()>no_object_timeout) { //no object in last 3 seconds
                    reset();
                    continue;
                }

                tf::Vector3 v=transform.getOrigin();
                double pan_err=-atan2(v.x(),v.z());
                double tilt_err=atan2(v.y(),v.z());

                if (first) {

                    pre_pan_err=pan_err;
                    pre_tilt_err=tilt_err;
                    first=false;


                }
                ros::Time now=ros::Time::now();
                dt=(now-pre_t);

                double pan_rate=pan_err*kp_pan+kd_pan*(pan_err-pre_pan_err)/dt.toSec();
                double tilt_rate=tilt_err*kp_tilt+kd_tilt*(tilt_err-pre_tilt_err)/dt.toSec();;

                if (pan_rate>max_rate) pan_rate=max_rate;
                else if(pan_rate<-max_rate) pan_rate=-max_rate;
                if (tilt_rate>max_rate) tilt_rate=max_rate;
                else if(tilt_rate<-max_rate) tilt_rate=-max_rate;

                if(fabs(pan_err)<target_tol) pan_rate=0;
                if(fabs(tilt_err)<target_tol) tilt_rate=0;


                pre_t=now;

                std_msgs::Float64MultiArray multiArray;
                double pan_cmd=pan_position+pan_rate*dt.toSec();
                double tilt_cmd=tilt_position+tilt_rate*dt.toSec();
                multiArray.data.push_back(pan_cmd); // pan (positive is left)
                multiArray.data.push_back(tilt_cmd); // tilt (positive is down)
                pan_tilt_pub.publish(multiArray);


            }
            catch (tf::TransformException ex) {
                 ROS_ERROR("Pan-Tilt tracking node error: %s",ex.what());
                reset();
            }
        }
        //
        ros::spinOnce();
        loopRate.sleep();
    }


    return 0;


}
