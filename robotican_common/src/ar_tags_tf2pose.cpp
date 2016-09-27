

#include <ros/ros.h>


#include <geometry_msgs/PoseStamped.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

ros::Publisher pub;

void chatterCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& markers)
{


if (markers->markers.size()==0) return;
  ROS_INFO("I heard");
geometry_msgs::PoseStamped msg;

ar_track_alvar_msgs::AlvarMarker m=markers->markers[0];
  msg.header.frame_id=m.header.frame_id;
  msg.header.stamp=m.header.stamp ;
  msg.pose=m.pose.pose;

  pub.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ar_tags_tf2pose_bridge");
    ros::NodeHandle nh;
ros::NodeHandle pn("~");

std::string object_id,object_tf;
pn.param<std::string>("object_id", object_id, "can");
pn.param<std::string>("object_tf", object_tf, "ar_marker_123");
std::string obj_topic="/detected_objects/"+object_id;
 pub = nh.advertise<geometry_msgs::PoseStamped>(obj_topic,10);
ros::Subscriber sub = nh.subscribe("ar_pose_marker", 1000, chatterCallback);




        ros::spin();

    return 0;


}
