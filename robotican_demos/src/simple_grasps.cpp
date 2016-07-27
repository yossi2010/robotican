#include <iostream>
#include <stdio.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>

#include <manipulation_msgs/GraspPlanning.h>

ros::Publisher goal_pub;

class PickAndPlaceTest{

protected:
    ros::NodeHandle node_handle;
    ros::ServiceClient planning_scene_diff_client;
    ros::ServiceClient grasp_planning_service;

public:
    PickAndPlaceTest(){
        planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();
         goal_pub=node_handle.advertise<geometry_msgs::PoseStamped>("pick_goal", 2, true);
    }

    ~PickAndPlaceTest(){
    }


    void executePick(){
        moveit::planning_interface::MoveGroup arm("arm");


        moveit_msgs::Grasp grasp;
        grasp.id = "grasp";

   grasp.pre_grasp_posture.joint_names.resize(2);
        grasp.pre_grasp_posture.joint_names[0]="left_finger_joint";
       grasp.pre_grasp_posture.joint_names[1]="right_finger_joint";
        grasp.pre_grasp_posture.points.resize(1);
        grasp.pre_grasp_posture.points[0].positions.resize(2);
        grasp.pre_grasp_posture.points[0].positions[0] = -0.57;
        grasp.pre_grasp_posture.points[0].positions[1] = 0.57;

        grasp.grasp_posture.joint_names.resize(2);
        grasp.grasp_posture.joint_names[0]="left_finger_joint";
       grasp.grasp_posture.joint_names[1]="right_finger_joint";
        grasp.grasp_posture.points.resize(1);
        grasp.grasp_posture.points[0].positions.resize(2);
        grasp.grasp_posture.points[0].positions[0] = 0.3;
        grasp.grasp_posture.points[0].positions[1] = -0.3;



        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = 0.4;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.8;
      //  pose.pose.orientation.x = 0.215;
      //  pose.pose.orientation.y = -0.674;
       // pose.pose.orientation.z = 0.215;
       // pose.pose.orientation.w = 0.674;
       // pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/2.0,0 );
        grasp.grasp_pose = pose;

       /* grasp.pre_grasp_approach.min_distance = 0.0;
        grasp.pre_grasp_approach.desired_distance = 0.0;
        grasp.pre_grasp_approach.direction.header.frame_id = "ft_fts_toolside";
        grasp.pre_grasp_approach.direction.vector.z = 1.0;

        grasp.post_grasp_retreat.min_distance = 0.0;
        grasp.post_grasp_retreat.desired_distance = 0.0;
        grasp.post_grasp_retreat.direction.header.frame_id = "ft_fts_toolside";
        grasp.post_grasp_retreat.direction.vector.z = -1.0;
*/
int err=0;
       // do {
            spawnObject(&arm);
             err=arm.pick("object");
            ROS_INFO("err code: %d",err);
       //     ros::Duration(2.0).sleep();
      //  } while (err!=1);
    }

    void spawnObject(moveit::planning_interface::MoveGroup *arm){
        moveit_msgs::ApplyPlanningScene srv;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;

        moveit_msgs::CollisionObject object;

        object.header.frame_id = "map";
        object.id = "object";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.05;
        primitive.dimensions[1] = 0.05;
        primitive.dimensions[2] = 0.20;


        geometry_msgs::Pose pose;
        pose.orientation.w=1;//=tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/2.0,0 );
        pose.position.x = 0.7;
        pose.position.y = 0.0;
        pose.position.z = 0.6+primitive.dimensions[2]/2;
        geometry_msgs::PoseStamped ps;
        ps.pose=pose;
        ps.header.stamp=ros::Time::now();
        ps.header.frame_id="map";
         goal_pub.publish(ps);

ROS_INFO("%f    %f    %f",pose.position.x,pose.position.y,pose.position.z);
        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);
        object.operation = object.ADD;
        planning_scene.world.collision_objects.push_back(object);

        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "PaPTest");
    PickAndPlaceTest testClass;
    testClass.executePick();
    ros::spin();
    return 0;
}
