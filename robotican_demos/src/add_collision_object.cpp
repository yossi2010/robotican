


#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit_msgs/WorkspaceParameters.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/CollisionObject.h>


ros::Publisher planning_scene_diff_publisher;
moveit_msgs::AttachedCollisionObject attached_object;

void update_table(geometry_msgs::Pose pose) {


    moveit_msgs::PlanningScene planning_scene1;
    attached_object.object.primitive_poses.clear();
    attached_object.object.primitive_poses.push_back(pose);

    planning_scene1.world.collision_objects.push_back(attached_object.object);
    planning_scene1.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene1);
    // ROS_INFO("Adding the table object into the world 2.");
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "pick_and_plce_node");


    ros::NodeHandle n;


     ros::Publisher object_in_map_pub_  = n.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
     while(object_in_map_pub_.getNumSubscribers() < 1)
     {
         ros::WallDuration sleep_t(0.5);
         sleep_t.sleep();
     }

     //add the cylinder into the collision space
     moveit_msgs::CollisionObject collision_object;
     collision_object.header.frame_id = "base_footprint";

     /* The id of the object is used to identify it. */
     collision_object.id = "box1";

     /* Define a box to add to the world. */
     shape_msgs::SolidPrimitive primitive;
     primitive.type = primitive.BOX;
     primitive.dimensions.resize(3);
     primitive.dimensions[0] = 0.2;
     primitive.dimensions[1] = 0.5;
     primitive.dimensions[2] = 0.01;

     /* A pose for the box (specified relative to frame_id) */
     geometry_msgs::Pose box_pose;
     box_pose.orientation.w = 1.0;
     box_pose.position.x = 0.5;
     box_pose.position.y = 0.2;
     box_pose.position.z = 0.8;

     collision_object.primitives.push_back(primitive);
     collision_object.primitive_poses.push_back(box_pose);
     collision_object.operation = collision_object.ADD;
     object_in_map_pub_.publish(collision_object);

     ROS_INFO("Should have published");


     /*
    planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    attached_object.link_name = "base_footprint";

    attached_object.object.header.frame_id = "base_footprint";

    attached_object.object.id = "table";



    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.5;
    primitive.dimensions[2] = 0.01;

    attached_object.object.primitives.push_back(primitive);

    attached_object.object.operation = attached_object.object.ADD;

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x=3;
    pose.position.y=0;
    pose.position.z=3;


update_table(pose);

*/


ros::spin();

    return 0;
}

