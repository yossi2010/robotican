#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit_msgs/WorkspaceParameters.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/SetBool.h>

void look_down();
bool set_collision_update(bool state);
void demoPick(moveit::planning_interface::MoveGroup &group);

ros::ServiceClient *uc_client_ptr;
ros::Publisher pub_controller_command;

void look_down() {

    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.push_back("head_pan_joint");
    traj.joint_names.push_back("head_tilt_joint");
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(1.0);
    std::vector<double> q_goal(2);
    q_goal[0]=0.0;
    q_goal[1]=0.4;
    traj.points[0].positions=q_goal;
    traj.points[0].velocities.push_back(0);
    traj.points[0].velocities.push_back(0);
    pub_controller_command.publish(traj);
}

void demoPick(moveit::planning_interface::MoveGroup &group)
{
 std::vector<moveit_msgs::Grasp> grasps;
  for (std::size_t i = 0 ; i < 1000 ; ++i)
  {
    geometry_msgs::PoseStamped p = group.getRandomPose();
    p.pose.orientation.x = 0;
   p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    moveit_msgs::Grasp g;
   g.grasp_pose = p;
  //  g.grasp_pose.pose.orientation.w=1;
  // p.header.frame_id="gripper_link";

    g.pre_grasp_approach.direction.vector.x = 1.0;

    g.pre_grasp_approach.min_distance = 0.02;
    g.pre_grasp_approach.desired_distance = 0.05;

    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.direction.header = p.header;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.2;


    g.pre_grasp_posture.joint_names.push_back("left_finger_joint");
    g.pre_grasp_posture.joint_names.push_back("right_finger_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(2);
    g.pre_grasp_posture.points[0].positions[0] = -0.57;
    g.pre_grasp_posture.points[0].positions[1] = 0.57;

    g.grasp_posture.joint_names.push_back("left_finger_joint");
    g.grasp_posture.joint_names.push_back("right_finger_joint");
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(2);
    g.grasp_posture.points[0].positions[0] = 0.3;
    g.grasp_posture.points[0].positions[1] = -0.3;

    g.allowed_touch_objects.push_back("wrist_link");
    g.allowed_touch_objects.push_back("left_finger_link");
    g.allowed_touch_objects.push_back("right_finger_link");

   grasps.push_back(g);
  }
  group.pick("can");
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "pick_and_plce_node");
    ros::AsyncSpinner spinner(2);

    ros::NodeHandle n;

    ROS_INFO("Hello");


     pub_controller_command = n.advertise<trajectory_msgs::JointTrajectory>("/pan_tilt_trajectory_controller/command", 2);

    ROS_INFO("Waiting for the moveit action server to come up");
    moveit::planning_interface::MoveGroup group("arm");


    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.5);
    group.setPlanningTime(5.0);
    group.setNumPlanningAttempts(10);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");



    spinner.start();
    std::string uc="/update_collision/object";
    ros::ServiceClient uc_client = n.serviceClient<std_srvs::SetBool>(uc);
    ROS_INFO("Waiting for update_collision service...");
    uc_client.waitForExistence();
    uc_client_ptr = &uc_client;
    set_collision_update(true);
    ROS_INFO("Looking down...");
    look_down();

    ROS_INFO("Ready!");
ros::Duration(2).sleep();

demoPick(group);

    //ros::spin();
ros::shutdown();
    return 0;
}


bool set_collision_update(bool state){
    std_srvs::SetBool srv;
    srv.request.data=state;
    if (uc_client_ptr->call(srv))
    {
        ROS_INFO("update_colision response: %s", srv.response.message.c_str());
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service /find_objects_node/update_colision");
        return false;
    }

}


