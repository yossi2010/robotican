#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>

void demoPick(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::Grasp> grasps;
  for (std::size_t i = 0 ; i < 20 ; ++i)
  {
    geometry_msgs::PoseStamped p = group.getRandomPose();
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    moveit_msgs::Grasp g;
    g.grasp_pose = p;
    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.direction.header = p.header;
    g.pre_grasp_approach.min_distance = 0.2;
    g.pre_grasp_approach.desired_distance = 0.4;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.27;
    g.pre_grasp_posture.joint_names.resize(1, "right_finger_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(1);
    g.pre_grasp_posture.points[0].positions[0] = 1;

    g.grasp_posture.joint_names.resize(1, "right_finger_joint");
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(1);
    g.grasp_posture.points[0].positions[0] = 0;

    grasps.push_back(g);
  }
  group.pick("box1", grasps);
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
  group.place("box1", loc);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);

  ros::AsyncSpinner spinner(1);
  spinner.start();

    moveit::planning_interface::MoveGroup group("arm");
 moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Add Collision Object
  /********************************************/
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = group.getEndEffectorLink();

    moveit_msgs::CollisionObject collision_object;

    //Define header, that is used for interpreting the poses
    collision_object.header.frame_id = group.getPlanningFrame();

    collision_object.id = "box1";

    //Define shape
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.04;
    primitive.dimensions[1] = 0.01;
    primitive.dimensions[2] = 0.2;

    //Define pose
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.3;
    box_pose.position.y = -0.4;
    box_pose.position.z = 0.1;

    //Define collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    //Define attached object
    attached_object.object = collision_object;

    //Add Object to the world
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    ROS_INFO("Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
    ros::Duration(2.0).sleep();

  // moveit::planning_interface::MoveGroup group(argc > 1 ? argv[1] : "right_arm");

  demoPlace(group);

  sleep(2);

  return 0;
}
