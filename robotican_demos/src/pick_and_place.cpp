


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/WorkspaceParameters.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;

robot_state::GroupStateValidityCallbackFn state_validity_callback_fn_;
const robot_model::JointModelGroup* joint_model_group;

bool isIKSolutionCollisionFree(robot_state::RobotState *joint_state,
                               const robot_model::JointModelGroup *joint_model_group,
                               const double *ik_solution);

moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr;

planning_scene::PlanningScenePtr *planning_scene_ptr;
robot_state::RobotStatePtr *robot_state_ptr;

tf::TransformListener *listener_ptr;
tf::MessageFilter<geometry_msgs::PoseStamped> * tf_filter_;


std::vector<moveit_msgs::CollisionObject> col_objects;

void update_table(geometry_msgs::Pose pose);
bool checkIK(geometry_msgs::PoseStamped pose);
void go(tf::Transform  dest);
bool gripper_cmd(double gap,double effort);
bool arm_cmd(geometry_msgs::PoseStamped target_pose1);
geometry_msgs::PoseStamped pre_grasp_pose(geometry_msgs::PoseStamped object);
geometry_msgs::PoseStamped lift_arm();

geometry_msgs::PoseStamped object_pose;


void pick_go_cb(std_msgs::Empty);
void button_go_cb(std_msgs::Empty);
void look_down();

GripperClient *gripperClient_ptr;

moveit::planning_interface::MoveGroup *group_ptr;
std::string object_name;

geometry_msgs::PoseStamped pick_pose;
double pick_yaw=0;
bool have_goal=false;
bool moving=false;
bool ready=false;
bool pub_can=true;


ros::Publisher goal_pub;
ros::Publisher pub_controller_command;

double wrist_distance_from_object=0.10;
geometry_msgs::PoseStamped moveit_goal;


geometry_msgs::PoseStamped lift_arm(){
    geometry_msgs::PoseStamped target_pose1;
    target_pose1.header.frame_id="map";
    target_pose1.header.stamp=ros::Time::now();
    target_pose1.pose.position.x = 0.5;
    target_pose1.pose.position.y =  0.0;
    target_pose1.pose.position.z =  0.9;
    target_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
    return target_pose1;
}

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

geometry_msgs::PoseStamped pre_grasp_pose(geometry_msgs::PoseStamped object){
    geometry_msgs::PoseStamped target_pose;


    tf::Vector3 v;
    v.setX(object.pose.position.x);
    v.setY(object.pose.position.y);
    v.setZ(object.pose.position.z);

    pick_yaw=atan2(v.y(),v.x());

    target_pose.header.frame_id=object.header.frame_id;
    target_pose.header.stamp=ros::Time::now();

    float away=wrist_distance_from_object/sqrt(v.x()*v.x()+v.y()*v.y());
    tf::Vector3 dest=v*(1-away);

    target_pose.pose.position.x = dest.x();
    target_pose.pose.position.y = dest.y();
    target_pose.pose.position.z =  v.z();
    target_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,pick_yaw );

    return target_pose;

}

void pick_go_cb(std_msgs::Empty) {
bool done=false;
    if (!moving) moving=true;
    ROS_INFO("Openning gripper...");
    if(gripper_cmd(0.14,0.0)) {
        ROS_INFO("Gripper is oppend, planning for pre-grasping..");
        ros::Duration(2).sleep();//wait for re-detection
        pick_pose=pre_grasp_pose(object_pose);
        if (arm_cmd(pick_pose)) {
            ROS_INFO("Arm planning is done, moving arm..");
            if(group_ptr->move()) {
                ROS_INFO("Ready to grasp");
                if(gripper_cmd(0.0,0.3)) {
                    ROS_INFO("Grasping is done");
                    pub_can=false;
                    std::vector<std::string> touch_links;
                    touch_links.push_back("left_finger_link");
                    touch_links.push_back("right_finger_link");
                    touch_links.push_back("wrist_link");
                    touch_links.push_back("gripper_link");
                   group_ptr->attachObject("can","gripper_link",touch_links);
                    ros::Duration(8).sleep(); //wait for attach
                    ROS_INFO("Lifting object...");
                    if (arm_cmd(lift_arm())) {
                        ROS_INFO("Arm planning is done, moving arm up..");
                        if (group_ptr->move()) {
                            ROS_INFO("Arm is up, placing on table...");
                            pick_pose.pose.position.z=pick_pose.pose.position.z+0.01;
                            pick_pose.pose.position.y=pick_pose.pose.position.y+0.1;
                            if (arm_cmd(pick_pose)) {
                                ROS_INFO("Arm planning is done, moving arm..");
                                if(group_ptr->move()) {
                                    ROS_INFO("Openning gripper...");
                                    if(gripper_cmd(0.14,0.0)) {
                                        ros::Duration(5).sleep(); //wait for deattach
                                        ROS_INFO("Lifting arm up...");
                                        if (arm_cmd(lift_arm())) {
                                             group_ptr->detachObject("can");
                                           //  std::vector<std::string> rem;
                                           //  rem.push_back("can");
                                           //  planning_scene_interface_ptr->removeCollisionObjects(rem);

                                            ROS_INFO("Arm planning is done, moving arm up..");
                                            if (group_ptr->move()) {
                                                ROS_INFO("Arm is up");
                                                ROS_INFO("Done!");
                                                done=true;

                                            }

                                        }

                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    if (!done) group_ptr->detachObject("can");
    moving=false;
    pub_can=true;
}


bool isIKSolutionCollisionFree(robot_state::RobotState *joint_state,
                               const robot_model::JointModelGroup *joint_model_group_,
                               const double *ik_solution)
{
    joint_state->setJointGroupPositions(joint_model_group_, ik_solution);
    bool result = !(*planning_scene_ptr)->isStateColliding(*joint_state, joint_model_group_->getName(),false);

    return result;
}

bool checkIK(geometry_msgs::PoseStamped pose) {

    // bool found_ik = (*robot_state_ptr)->setFromIK(joint_model_group, pose.pose, 10,1);
    bool found_ik = (*robot_state_ptr)->setFromIK(joint_model_group, pose.pose, 10,3.0, state_validity_callback_fn_);
    std::printf("IK %d: [%f , %f , %f] [%f , %f , %f , %f]\n",found_ik,pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
    return found_ik;
}

bool arm_cmd( geometry_msgs::PoseStamped target_pose1) {


    group_ptr->setStartStateToCurrentState();

    if (!have_goal) have_goal=true;

    moveit::planning_interface::MoveGroup::Plan my_plan;
    double dz[]={0};//{0, 0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
    double dy[]={0};//{0, 0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
    double dx[]={0};//{0, 0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
    double dY[]={0, 0.05, -0.05};//, 0.04, -0.04 ,0.18, -0.18};
    double z=target_pose1.pose.position.z;
    double x=target_pose1.pose.position.x;
    double y=target_pose1.pose.position.y;
    for (int n=0;n<sizeof(dx)/sizeof(double);n++) {
        for (int m=0;m<sizeof(dy)/sizeof(double);m++) {

            for (int i=0;i<sizeof(dz)/sizeof(double);i++) {
                for (int j=0;j<sizeof(dY)/sizeof(double);j++) {
                    target_pose1.pose.position.z=z+dz[i];
                    target_pose1.pose.position.x=x+dx[n];
                    target_pose1.pose.position.y=y+dy[m];

                    target_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,pick_yaw+dY[j] );
                    //     tf::Quaternion q( target_pose1.pose.orientation.x,  target_pose1.pose.orientation.y,  target_pose1.pose.orientation.z, target_pose1.pose.orientation.w);
                    //      double roll, pitch, yaw;
                    //             tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                    //ROS_INFO("%f",yaw*180/M_PI);

                    if (checkIK(target_pose1)) {
                        goal_pub.publish(target_pose1);


                        group_ptr->setPoseTarget(target_pose1);

                        bool success = group_ptr->plan(my_plan);
                        ROS_INFO("Moveit plan %s",success?"SUCCESS":"FAILED");
                        if (success) return true;
                    }
                }
            }
        }
    }
    ROS_INFO("FAILED to find plan");
    return false;
}

bool gripper_cmd(double gap,double effort) {

    control_msgs::GripperCommandGoal openGoal;

    openGoal.command.position = gap;
    openGoal.command.max_effort = effort;
    gripperClient_ptr->sendGoal(openGoal);
    ROS_INFO("Sent gripper goal");
    gripperClient_ptr->waitForResult();

    if(gripperClient_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Gripper done");
        return true;
    }
    else {
        ROS_ERROR("Gripper fault");
        // return false;
    }
    return false;
}

//  Callback to register with tf::MessageFilter to be called when transforms are available
void msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr)
{
    if (!ready) return;
    try
    {
        listener_ptr->transformPose("base_footprint", *point_ptr, object_pose);
        object_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
        if (!moving) {
            geometry_msgs::PoseStamped table=object_pose;
            table.pose.position.z-=0.05;
            update_table(table.pose);

            // tf::Quaternion q( pose_in_map.pose.orientation.x,  pose_in_map.pose.orientation.y,  pose_in_map.pose.orientation.z, pose_in_map.pose.orientation.w);
            // double roll, pitch, yaw;
            //tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            // ROS_INFO("%f  %f  %f",roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);
            bool ik=checkIK(pre_grasp_pose(object_pose));

        }
        //printf("point of object in frame of base_footprint Position(x:%f y:%f z:%f)\n", object_pose.pose.position.x, object_pose.pose.position.y,object_pose.pose.position.z);
    }
    catch (tf::TransformException &ex)
    {
        printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
}

void update_table(geometry_msgs::Pose pose) {

    col_objects[0].primitive_poses.clear();
    col_objects[0].primitive_poses.push_back(pose);


    pose.position.z+=0.07;
    pose.position.x+=0.015;
    col_objects[1].primitive_poses.clear();
    if(pub_can) col_objects[1].primitive_poses.push_back(pose);

    planning_scene_interface_ptr->addCollisionObjects(col_objects);


}


int main(int argc, char **argv) {

    ros::init(argc, argv, "pick_and_plce_node");
    ros::AsyncSpinner spinner(2);


    ros::NodeHandle pn("~");
    ros::NodeHandle n;

    ROS_INFO("Hello");

    pn.param<double>("wrist_distance_from_object", wrist_distance_from_object, 0.02);
    pn.param<std::string>("object_name", object_name, "kinect2_object");
    std::string topic="/detected_objects/"+object_name;



    ROS_INFO("Waiting for the moveit action server to come up");
    moveit::planning_interface::MoveGroup group("arm");
    group_ptr=&group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface_ptr=&planning_scene_interface;


    // group.allowReplanning(true);
    group.setMaxVelocityScalingFactor(0.05);
    group.setMaxAccelerationScalingFactor(0.01);
    group.setPlanningTime(15.0);
    group.setNumPlanningAttempts(30);
    group.setPlannerId("RRTConnectkConfigDefault");
    // group.setPlannerId("RRTstarkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");


    ros::Subscriber pick_sub = n.subscribe("pick_go", 1, pick_go_cb);


    goal_pub=n.advertise<geometry_msgs::PoseStamped>("pick_moveit_goal", 2, true);
    pub_controller_command = n.advertise<trajectory_msgs::JointTrajectory>("/pan_tilt_trajectory_controller/command", 2);


    tf::TransformListener listener;
    listener_ptr=&listener;
    message_filters::Subscriber<geometry_msgs::PoseStamped> point_sub_;
    point_sub_.subscribe(n, topic, 10);
    tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(point_sub_, listener, "base_footprint", 10);
    tf_filter_->registerCallback( boost::bind(msgCallback, _1) );



    GripperClient gripperClient("/gripper_controller/gripper_cmd", true);
    //wait for the gripper action server to come up
    while ((!gripperClient.waitForServer(ros::Duration(5.0)))&&(ros::ok())){
        ROS_INFO("Waiting for the /gripper_controller/gripper_cmd action server to come up");
    }
    gripperClient_ptr=&gripperClient;


    state_validity_callback_fn_ = boost::bind(&isIKSolutionCollisionFree, _1, _2, _3);



    /* Get a shared pointer to the model */
    robot_model::RobotModelConstPtr robot_model = group.getRobotModel();// robot_model_loader.getModel();

    /* Create a robot state*/
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));

    robot_state_ptr=&robot_state;

    std::string group_name="arm";
    if(!robot_model->hasJointModelGroup(group_name))
        ROS_FATAL("Invalid group name: %s", group_name.c_str());

    joint_model_group = robot_model->getJointModelGroup(group_name);
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    planning_scene_ptr=&planning_scene;


    moveit_msgs::CollisionObject table_collision_object;
    table_collision_object.header.frame_id = "base_footprint";
    table_collision_object.id = "table";
    shape_msgs::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[0] = 0.2;
    table_primitive.dimensions[1] = 0.5;
    table_primitive.dimensions[2] = 0.01;
    table_collision_object.primitives.push_back(table_primitive);
    table_collision_object.operation = table_collision_object.ADD;
    col_objects.push_back(table_collision_object);

    moveit_msgs::CollisionObject can_collision_object;
    can_collision_object.header.frame_id = "base_footprint";
    can_collision_object.id = "can";
    shape_msgs::SolidPrimitive object_primitive;
    object_primitive.type = object_primitive.CYLINDER;
    object_primitive.dimensions.resize(2);
    object_primitive.dimensions[0] = 0.1;
    object_primitive.dimensions[1] = 0.02;
    can_collision_object.primitives.push_back(object_primitive);
    can_collision_object.operation = can_collision_object.ADD;
    col_objects.push_back(can_collision_object);

    spinner.start();

    ros::Duration(2.0).sleep();
    ROS_INFO("Looking down...");
    look_down();
    if (arm_cmd(lift_arm())) {
        ROS_INFO("Arm planning is done, moving arm up..");
        if (group_ptr->move()) {
            ROS_INFO("Arm is up");
        }
    }

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "gripper_link";
    ocm.header.frame_id = "gripper_link";
    ocm.orientation.x = 0;
    ocm.orientation.y = 0;
    ocm.orientation.z = 0;
    ocm.orientation.w = 1;
    ocm.absolute_x_axis_tolerance = 5.0*M_PI/180.0;
    ocm.absolute_y_axis_tolerance = 5.0*M_PI/180.0;
    ocm.absolute_z_axis_tolerance = M_PI;
    ocm.weight = 1.0;
    moveit_msgs::Constraints constr;
    constr.orientation_constraints.push_back(ocm);
    group.setPathConstraints(constr);


    ready=true;
    ROS_INFO("Ready!");
    while (ros::ok())
    {



    }

    return 0;
}

