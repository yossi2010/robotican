#include <iostream>
#include <stdio.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>

#include <manipulation_msgs/GraspPlanning.h>

class PickAndPlaceTest{

protected:
    ros::NodeHandle node_handle;
    ros::ServiceClient planning_scene_diff_client;
    ros::ServiceClient grasp_planning_service;

public:
    PickAndPlaceTest(){
        planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();
    }

    ~PickAndPlaceTest(){
    }

    void executePick(){
        moveit::planning_interface::MoveGroup arm("arm");
        spawnObject();
/*
        moveit_msgs::Grasp grasp;
        grasp.id = "grasp";
        grasp.pre_grasp_posture.joint_names.resize(1, "s_model_finger_1_joint_1");
        grasp.pre_grasp_posture.points.resize(1);
        grasp.pre_grasp_posture.points[0].positions.resize(1);
        grasp.pre_grasp_posture.points[0].positions[0] = 0.0495296;
        grasp.grasp_posture.joint_names.resize(1, "s_model_finger_1_joint_1");
        grasp.grasp_posture.points.resize(1);
        grasp.grasp_posture.points[0].positions.resize(1);
        grasp.grasp_posture.points[0].positions[0] = 0.924553;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.pose.position.x = 0.736;
        pose.pose.position.y = 0.709;
        pose.pose.position.z = 0.14;
        pose.pose.orientation.x = 0.215;
        pose.pose.orientation.y = -0.674;
        pose.pose.orientation.z = 0.215;
        pose.pose.orientation.w = 0.674;
        grasp.grasp_pose = pose;
        grasp.pre_grasp_approach.min_distance = 0.0;
        grasp.pre_grasp_approach.desired_distance = 0.0;
        grasp.pre_grasp_approach.direction.header.frame_id = "ft_fts_toolside";
        grasp.pre_grasp_approach.direction.vector.z = 1.0;
        grasp.post_grasp_retreat.min_distance = 0.0;
        grasp.post_grasp_retreat.desired_distance = 0.0;
        grasp.post_grasp_retreat.direction.header.frame_id = "ft_fts_toolside";
        grasp.post_grasp_retreat.direction.vector.z = -1.0;
*/

        arm.pick("object");
    }

    void spawnObject(){
        moveit_msgs::ApplyPlanningScene srv;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;

        moveit_msgs::CollisionObject object;

        object.header.frame_id = "/map";
        object.id = "object";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.05;
        primitive.dimensions[1] = 0.05;
        primitive.dimensions[2] = 0.20;

        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        pose.position.x = 0.55;
        pose.position.y = 0.0;
        pose.position.z = 0.9+primitive.dimensions[2]/2;

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
/*
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>

void demoPick(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::Grasp> grasps;

  for (std::size_t i = 0 ; i < 200 ; ++i)
  {
    geometry_msgs::PoseStamped p = group.getRandomPose();
//    geometry_msgs::PoseStamped p;// = group.getRandomPose();
//    p.pose.position.x=0.4;
 //   p.pose.position.z=0.9;
//p.header.frame_id="base_footprint";
    p.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

    moveit_msgs::Grasp g;
    g.grasp_pose = p;
    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.post_grasp_retreat.direction.vector.z = 1.0;

    g.post_grasp_retreat.direction.header = p.header;
    g.pre_grasp_approach.min_distance = 0.01;
    g.pre_grasp_approach.desired_distance = 0.03;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.27;

    g.pre_grasp_posture.joint_names.resize(2);
     g.pre_grasp_posture.joint_names.push_back("right_finger_joint");
   g.pre_grasp_posture.joint_names.push_back("left_finger_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(2);
    g.pre_grasp_posture.points[0].positions[0] = 0.57;
 g.pre_grasp_posture.points[0].positions[1] = -0.57;


 g.grasp_posture.joint_names.resize(2);
  g.grasp_posture.joint_names.push_back("right_finger_joint");
g.grasp_posture.joint_names.push_back("left_finger_joint");
 g.grasp_posture.points.resize(1);
 g.grasp_posture.points[0].positions.resize(2);
 g.grasp_posture.points[0].positions[0] = -0.3;
g.grasp_posture.points[0].positions[1] = 0.3;


    grasps.push_back(g);
}
  group.pick("box",grasps);
}

void demoPlace(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;
  for (std::size_t i = 0 ; i < 20 ; ++i)
  {
    geometry_msgs::PoseStamped p = group.getRandomPose();
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    moveit_msgs::PlaceLocation g;
    g.place_pose = p;
    g.pre_place_approach.direction.vector.x = 1.0;
    g.post_place_retreat.direction.vector.z = 1.0;
    g.post_place_retreat.direction.header = p.header;
    g.pre_place_approach.min_distance = 0.2;
    g.pre_place_approach.desired_distance = 0.4;
    g.post_place_retreat.min_distance = 0.1;
    g.post_place_retreat.desired_distance = 0.27;

    g.post_place_posture.joint_names.resize(1, "right_finger_joint");
    g.post_place_posture.points.resize(1);
    g.post_place_posture.points[0].positions.resize(1);
    g.post_place_posture.points[0].positions[0] = 0;

    loc.push_back(g);
  }
  group.place("box", loc);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // moveit::planning_interface::MoveGroup group(argc > 1 ? argv[1] : "right_arm");
  moveit::planning_interface::MoveGroup group("arm");

  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "map";

  attached_object.object.header.frame_id = "map";

  attached_object.object.id = "box";


  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
pose.position.x=0.6;
pose.position.z=0.7;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

attached_object.object.operation = attached_object.object.ADD;

ROS_INFO("Adding the object into the world.");
moveit_msgs::PlanningScene planning_scene;
planning_scene.world.collision_objects.push_back(attached_object.object);
planning_scene.is_diff = true;
planning_scene_diff_publisher.publish(planning_scene);

  demoPick(group);


ros::spin();
  return 0;
}
*/
