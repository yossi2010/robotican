


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

tf::TransformListener *listener_ptr;

move_base_msgs::MoveBaseGoal get_pre_pick_pose();
bool base_cmd(move_base_msgs::MoveBaseGoal goal);

MoveBaseClient *moveBaseClient_ptr;

bool moving=false;

double base_distance_from_object=0.55;

ros::Time obj_t;
geometry_msgs::PoseStamped map_object_pose;

move_base_msgs::MoveBaseGoal get_pre_pick_pose() {
    tf::Transform dest_transform;
    move_base_msgs::MoveBaseGoal goal;
    try{

        tf::StampedTransform transform_base;
        listener_ptr->lookupTransform("map", "base_link", ros::Time(0), transform_base);

        tf::Vector3 v_obj(map_object_pose.pose.position.x,map_object_pose.pose.position.y,map_object_pose.pose.position.z);

        tf::Vector3 v_base =transform_base.getOrigin();

        tf::Vector3 v=v_obj-v_base;
        double yaw=atan2(v.y(),v.x());
        double away=base_distance_from_object/sqrt(v.x()*v.x()+v.y()*v.y());
        tf::Vector3 dest=v_base+v*(1-away);

        dest_transform.setOrigin( dest );
        dest.setZ(0);
        tf::Quaternion q;
        q.setRPY(0.0, 0, yaw);
        dest_transform.setRotation(q);

        std::cout<< map_object_pose.pose.position<<std::endl;

    }

    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return goal;
    }

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x=dest_transform.getOrigin().x();
    goal.target_pose.pose.position.y=dest_transform.getOrigin().y();
    goal.target_pose.pose.position.z=0;
    goal.target_pose.pose.orientation.x=dest_transform.getRotation().x();
    goal.target_pose.pose.orientation.y=dest_transform.getRotation().y();
    goal.target_pose.pose.orientation.z=dest_transform.getRotation().z();
    goal.target_pose.pose.orientation.w=dest_transform.getRotation().w();
    return goal;
}



bool drive_go_cb(std_srvs::Trigger::Request  &req,
                 std_srvs::Trigger::Response &res)
{
    double dt=(ros::Time::now()-obj_t).toSec();
    if (dt >2.0) {
        ROS_ERROR("No object found during the last 2 seconds");
        res.message="No object found during the last 2 seconds";
        res.success=false;
        return true;
    }
    if (!moving) moving=true;
    move_base_msgs::MoveBaseGoal goal=get_pre_pick_pose();
    if (base_cmd(goal)) {
        ROS_INFO("Reached position");
        res.message="Reached pre-picking position";
        res.success=true;
    }
    moving=false;
    return true;
}



void obj_msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr)
{
    if (moving) return;
    try
    {
        listener_ptr->transformPose("map", *point_ptr, map_object_pose);
        obj_t=ros::Time::now();
    }
    catch (tf::TransformException &ex)
    {
        printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
}

bool base_cmd(move_base_msgs::MoveBaseGoal goal) {

    ROS_INFO("[%s]: Sending goal", ros::this_node::getName().c_str());

    moveBaseClient_ptr->sendGoal(goal);
    moveBaseClient_ptr->waitForResult();

    if(moveBaseClient_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        return true;
    }
    else {
        ROS_ERROR("[%s]: Navigation failed ", ros::this_node::getName().c_str());
        return false;
    }
}
int main(int argc, char **argv) {

    ros::init(argc, argv, "drive2object_node");
   // ros::AsyncSpinner spinner(2);
    //spinner.start();

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    std::string object_name;
    pn.param<double>("base_distance_from_object", base_distance_from_object, 0.55);
    pn.param<std::string>("object_name", object_name, "object");
    std::string obj_topic="/detected_objects/"+object_name;


    MoveBaseClient moveBaseClient("move_base", true);
    //wait for the action server to come up
    while(!moveBaseClient.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    moveBaseClient_ptr=&moveBaseClient;



    ros::ServiceServer service = n.advertiseService("drive2object_go", drive_go_cb);

    tf::TransformListener listener;
    listener_ptr=&listener;

    message_filters::Subscriber<geometry_msgs::PoseStamped> point_sub;
    point_sub.subscribe(n, obj_topic, 10);

    tf::MessageFilter<geometry_msgs::PoseStamped> tf_filter(point_sub, listener, "map", 10);
    tf_filter.registerCallback( boost::bind(obj_msgCallback, _1) );


    ros::Rate r(50); // 50 hz




ros::spin();

    return 0;
}

