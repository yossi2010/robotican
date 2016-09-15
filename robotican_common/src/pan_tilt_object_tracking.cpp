

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;


void callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
PointHeadClient* point_head_client;


void callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{

//the goal message we will be sending
pr2_controllers_msgs::PointHeadGoal goal;

//the target point, expressed in the requested frame
geometry_msgs::PointStamped point;
point.header.frame_id = pose->header.frame_id;
point.point.x = pose->pose.position.x; point.point.y = pose->pose.position.y; point.point.z = pose->pose.position.z;
goal.target = point;

//we are pointing the high-def camera frame
//(pointing_axis defaults to X-axis)
goal.pointing_frame = "kinect2_depth_frame";
goal.pointing_axis.x = 1;
goal.pointing_axis.y = 0;
goal.pointing_axis.z = 0;

//take at least 0.5 seconds to get there
goal.min_duration = ros::Duration(0.5);

//and go no faster than 0.2 rad/s
goal.max_velocity = 0.2;

//send the goal
point_head_client->sendGoal(goal);

//wait for it to get there (abort after 2 secs to prevent getting stuck)
point_head_client->waitForResult(goal.min_duration );

}




int main(int argc, char **argv) {
    ros::init(argc, argv, "pan_tilt_object_trackking");
    ros::NodeHandle nh;
ros::NodeHandle pn("~");

std::string object_id;
pn.param<std::string>("object_id", object_id, "can");
std::string obj_topic="/detected_objects/"+object_id;
ros::Subscriber obj_sub = nh.subscribe(obj_topic, 1, callback);


//Initialize the client for the Action interface to the head controller
point_head_client = new PointHeadClient("/pan_tilt_trajectory_controller/point_head_action", true);

//wait for head controller action server to come up
while(!point_head_client->waitForServer(ros::Duration(5.0))){
  ROS_INFO("Waiting for the point_head_action server to come up");
}

    ROS_INFO("Ready to track!");


        ros::spin();


    return 0;


}
