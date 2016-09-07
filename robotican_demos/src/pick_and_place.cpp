


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Empty.h>
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
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;


void apply_floor_constraints();
void gripper_constraints(bool state);
void head_msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr);
void arm_msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr);
void update_collision_objects(geometry_msgs::Pose pose);
bool checkIK(geometry_msgs::PoseStamped pose);
void go(tf::Transform  dest);
bool gripper_cmd(double gap,double effort);
bool arm_cmd(geometry_msgs::PoseStamped target_pose1);
bool arm_plan_and_exec(geometry_msgs::PoseStamped target_pose1);
geometry_msgs::PoseStamped pre_grasp_pose(geometry_msgs::PoseStamped object);
bool plan_arm(std::string pose);
void pick_go_cb(std_msgs::Empty);
void button_go_cb(std_msgs::Empty);
void look_down();

robot_state::GroupStateValidityCallbackFn state_validity_callback_fn_;
const robot_model::JointModelGroup* joint_model_group;
bool isIKSolutionCollisionFree(robot_state::RobotState *joint_state,
                               const robot_model::JointModelGroup *joint_model_group,
                               const double *ik_solution);

void addStaticObjects(const moveit::planning_interface::PlanningSceneInterface &planning_scene_interface);

void setStartConstraints(moveit::planning_interface::MoveGroup &group);

bool cartesianPathExecution(moveit_msgs::RobotTrajectory &trajectory);

moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr;
planning_scene::PlanningScenePtr *planning_scene_ptr;
robot_state::RobotStatePtr *robot_state_ptr;
tf::TransformListener *listener_ptr;
tf::MessageFilter<geometry_msgs::PoseStamped> *head_tf_filter_,*arm_tf_filter_;
std::vector<moveit_msgs::CollisionObject> col_objects, staticObjects;
geometry_msgs::PoseStamped head_object_pose,arm_object_pose;
GripperClient *gripperClient_ptr;
moveit::planning_interface::MoveGroup *group_ptr;

double pick_yaw=0;
bool have_goal=false;
bool moving=false;
bool ready=false;
bool pub_can=true;
double wrist_distance_from_object;
geometry_msgs::PoseStamped moveit_goal;

double MaxAccelerationScalingFactor,MaxVelocityScalingFactor;

ros::Publisher pick_pub,head_object_pub,arm_object_pub;
ros::Publisher pub_controller_command;



bool plan_arm(std::string pose){
    group_ptr->setNamedTarget(pose);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    return group_ptr->plan(my_plan);

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
    geometry_msgs::PoseStamped target_pose=object;


    tf::Vector3 v;
    v.setX(object.pose.position.x);
    v.setY(object.pose.position.y);
    v.setZ(object.pose.position.z);

    pick_yaw=atan2(v.y(),v.x());


    float away=wrist_distance_from_object/sqrt(v.x()*v.x()+v.y()*v.y());
    tf::Vector3 dest=v*(1-away);

    target_pose.pose.position.x = dest.x();
    target_pose.pose.position.y = dest.y();
    target_pose.pose.position.z +=0;
    target_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,pick_yaw );
    pick_pub.publish(target_pose);

    return target_pose;

}



void pick_go_cb(std_msgs::Empty) {
    bool attached=false;
    if (!moving) moving=true;
    ROS_INFO("Openning gripper...");
    if(gripper_cmd(0.14,0.0)) {
        ROS_INFO("Gripper is oppend, planning for pre-grasping..");
        ros::Duration(2).sleep();//wait for re-detection
        geometry_msgs::PoseStamped pick_pose = pre_grasp_pose(head_object_pose);
        geometry_msgs::PoseStamped prePick = pick_pose;
        prePick.pose.position.z += 0.1;

        std::vector<geometry_msgs::Pose> wayPointsForArm2ObjPath;
        wayPointsForArm2ObjPath.push_back(prePick.pose);
        wayPointsForArm2ObjPath.push_back(pick_pose.pose);

        moveit_msgs::RobotTrajectory arm2ObjPath;
        double fractionArm2ObjPath = group_ptr->computeCartesianPath(wayPointsForArm2ObjPath,
                                                                     0.01,  // eef_step
                                                                     0.0,   // jump_threshold
                                                                     arm2ObjPath);


        if ((fractionArm2ObjPath > 0.8) && (cartesianPathExecution(arm2ObjPath))) {
            if (gripper_cmd(0.01, 0.4)) {
                ROS_INFO("Grasping is done");
                gripper_constraints(true);
                pub_can = false;
                std::vector<std::string> touch_links;
                touch_links.push_back("left_finger_link");
                touch_links.push_back("right_finger_link");
                touch_links.push_back("wrist_link");
                touch_links.push_back("gripper_link");
                group_ptr->attachObject("can", "gripper_link", touch_links);
                attached = true;
                ros::Duration(8).sleep(); //wait for attach
                ROS_INFO("Lifting object...");

                std::vector<geometry_msgs::Pose> wayPointsForArmLiftPath;
                pick_pose.pose.position.z += 0.10;
                wayPointsForArmLiftPath.push_back(pick_pose.pose);
                pick_pose.pose.position.y += 0.15;
                wayPointsForArmLiftPath.push_back(pick_pose.pose);
                pick_pose.pose.position.z -= 0.07;
                wayPointsForArmLiftPath.push_back(pick_pose.pose);
                moveit_msgs::RobotTrajectory armLiftPath;
                double fractionArmLiftPath = group_ptr->computeCartesianPath(wayPointsForArmLiftPath,
                                                                             0.01,  // eef_step
                                                                             0.0,   // jump_threshold
                                                                             armLiftPath);


                if((fractionArmLiftPath > 0.8) && (cartesianPathExecution(armLiftPath))) {
                    if(gripper_cmd(0.14, 0.0)) {
                        ros::Duration(5).sleep(); //wait for deattach
                        ROS_INFO("Lifting arm up...");
                        gripper_constraints(false);


                        group_ptr->detachObject("can");
                        attached = false;


                        if (plan_arm("pre_grasp2")) {
                            group_ptr->detachObject("can");
                            attached = false;
                            ROS_INFO("Arm planning is done, moving arm up..");
                            if (group_ptr->move()) {
                                ROS_INFO("Arm is up");
                                ROS_INFO("Done!");
                            }
                        }
                    }



                }
            }
        }


    }
    if (attached) group_ptr->detachObject("can");
    moving=false;
    pub_can=true;
}

bool cartesianPathExecution(moveit_msgs::RobotTrajectory &trajectory) {
    group_ptr->setStartStateToCurrentState();

    robot_trajectory::RobotTrajectory rt(group_ptr->getCurrentState()->getRobotModel(), "arm");
    rt.setRobotTrajectoryMsg(*group_ptr->getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    bool success = iptp.computeTimeStamps(rt,MaxVelocityScalingFactor,MaxAccelerationScalingFactor);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
    rt.getRobotTrajectoryMsg(trajectory);

    moveit::planning_interface::MoveGroup::Plan cartesianPathPlan;
    cartesianPathPlan.trajectory_ = trajectory;
    return group_ptr->execute(cartesianPathPlan);
}

bool arm_plan_and_exec(geometry_msgs::PoseStamped target_pose1) {


    group_ptr->setStartStateToCurrentState();

    if (!have_goal) have_goal=true;

    moveit::planning_interface::MoveGroup::Plan my_plan;
    double dz[]={0};//{0, 0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
    double dy[]={0};//{0, 0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
    double dx[]={0};//{0, 0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
    double dY[]={0};//, 0.04, -0.04 ,0.18, -0.18};
    double z=target_pose1.pose.position.z;
    double x=target_pose1.pose.position.x;
    double y=target_pose1.pose.position.y;
    tf::Quaternion q( target_pose1.pose.orientation.x,  target_pose1.pose.orientation.y,  target_pose1.pose.orientation.z, target_pose1.pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    //ROS_INFO("%f",yaw*180/M_PI);

    for (int n=0;n<sizeof(dx)/sizeof(double);n++) {
        for (int m=0;m<sizeof(dy)/sizeof(double);m++) {

            for (int i=0;i<sizeof(dz)/sizeof(double);i++) {
                for (int j=0;j<sizeof(dY)/sizeof(double);j++) {
                    target_pose1.pose.position.z=z+dz[i];
                    target_pose1.pose.position.x=x+dx[n];
                    target_pose1.pose.position.y=y+dy[m];

                    target_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw+dY[j] );


                    if (checkIK(target_pose1)) {
                        pick_pub.publish(target_pose1);


                        group_ptr->setPoseTarget(target_pose1);

                        bool success = group_ptr->plan(my_plan);

                        ROS_INFO("Moveit plan %s",success?"SUCCESS":"FAILED");
                        if (success && group_ptr->execute(my_plan)) {
                            return true;
                        }
                    }
                }
            }
        }
    }
    ROS_INFO("FAILED to find plan");
    return false;
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
    double dY[]={0};//, 0.04, -0.04 ,0.18, -0.18};
    double z=target_pose1.pose.position.z;
    double x=target_pose1.pose.position.x;
    double y=target_pose1.pose.position.y;
    tf::Quaternion q( target_pose1.pose.orientation.x,  target_pose1.pose.orientation.y,  target_pose1.pose.orientation.z, target_pose1.pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    //ROS_INFO("%f",yaw*180/M_PI);

    for (int n=0;n<sizeof(dx)/sizeof(double);n++) {
        for (int m=0;m<sizeof(dy)/sizeof(double);m++) {

            for (int i=0;i<sizeof(dz)/sizeof(double);i++) {
                for (int j=0;j<sizeof(dY)/sizeof(double);j++) {
                    target_pose1.pose.position.z=z+dz[i];
                    target_pose1.pose.position.x=x+dx[n];
                    target_pose1.pose.position.y=y+dy[m];

                    target_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw+dY[j] );


                    if (checkIK(target_pose1)) {
                        pick_pub.publish(target_pose1);


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
void head_msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr)
{
    if ((!ready)||(moving)) return;
    try
    {
        listener_ptr->transformPose("base_footprint", *point_ptr, head_object_pose);
        head_object_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
        head_object_pose.header.frame_id="base_footprint";
        head_object_pose.header.stamp=ros::Time::now();

        update_collision_objects(head_object_pose.pose);
        bool ik=checkIK(pre_grasp_pose(head_object_pose));
        head_object_pub.publish(head_object_pose);
    }
    catch (tf::TransformException &ex)
    {
        printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
}

//  Callback to register with tf::MessageFilter to be called when transforms are available
void arm_msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr)
{
    if ((!ready)||(moving)) return;
    try
    {
        listener_ptr->transformPose("base_footprint", *point_ptr, arm_object_pose);
        arm_object_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
        arm_object_pose.header.frame_id="base_footprint";
        arm_object_pose.header.stamp=ros::Time::now();

        // update_collision_objects(arm_object_pose.pose);
        //bool ik=checkIK(pre_grasp_pose(arm_object_pose));
        arm_object_pub.publish(head_object_pose);
    }
    catch (tf::TransformException &ex)
    {
        printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
}

void update_collision_objects(geometry_msgs::Pose pose) {

    geometry_msgs::Pose table=pose;
    table.position.z-=0.1;
    col_objects[0].primitive_poses.clear();
    col_objects[0].primitive_poses.push_back(table);

    col_objects[1].primitive_poses.clear();
    if(pub_can) col_objects[1].primitive_poses.push_back(pose);

    planning_scene_interface_ptr->addCollisionObjects(col_objects);


}


void gripper_constraints(bool apply) {

    moveit_msgs::Constraints c=group_ptr->getPathConstraints();

    if ((apply)&&(c.orientation_constraints.size()==0)) {

        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = "gripper_link";
        ocm.header.frame_id = "gripper_link";
        ocm.orientation.x = 0;
        ocm.orientation.y = 0;
        ocm.orientation.z = 0;
        ocm.orientation.w = 1;
        ocm.absolute_x_axis_tolerance = 10.0*M_PI/180.0;
        ocm.absolute_y_axis_tolerance = 10.0*M_PI/180.0;
        ocm.absolute_z_axis_tolerance = M_PI;
        ocm.weight = 1.0;
        moveit_msgs::Constraints constr;
        constr.orientation_constraints.push_back(ocm);
        group_ptr->setPathConstraints(constr);
        ROS_INFO("Applying gripper orientation constraints");
    }
    else if (!apply) {
        group_ptr->clearPathConstraints();
        ROS_INFO("Clearing gripper orientation constraints");
    }
}

void apply_floor_constraints() {

    moveit_msgs::Constraints c=group_ptr->getPathConstraints();
    moveit_msgs::PositionConstraint pc;


    moveit_msgs::BoundingVolume reg;
    shape_msgs::SolidPrimitive prim;
    prim.type = prim.BOX;
    prim.dimensions.resize(3);
    prim.dimensions[prim.BOX_X] = 4.0;
    prim.dimensions[prim.BOX_Y] = 4.0;
    prim.dimensions[prim.BOX_Z] = 4.0;

    geometry_msgs::Pose pose;
    pose.position.z=2.05;
    pose.orientation.w = 1;

    reg.primitives.push_back(prim);
    reg.primitive_poses.push_back(pose);
    pc.header.stamp = ros::Time::now();

    pc.constraint_region = reg;
    pc.link_name = "gripper_link";
    pc.header.frame_id = "base_footprint";
    pc.weight = 1.0;

    moveit_msgs::Constraints constr;
    constr.position_constraints.push_back(pc);

    group_ptr->setPathConstraints(constr);
    ROS_INFO("Applying floor position constraints");

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "pick_and_plce_node");
    ros::AsyncSpinner spinner(2);


    ros::NodeHandle pn("~");
    ros::NodeHandle n;

    ROS_INFO("Hello");

    pn.param<double>("wrist_distance_from_object", wrist_distance_from_object, 0.03);
    std::string object_name_head_camera,object_name_arm_camera;
    pn.param<std::string>("object_name_head_camera", object_name_head_camera, "kinect2_object");
    std::string head_topic="/detected_objects/"+object_name_head_camera;
    pn.param<std::string>("object_name_arm_camera", object_name_arm_camera, "sr300_object");
    std::string arm_topic="/detected_objects/"+object_name_arm_camera;


    pn.param<double>("MaxAccelerationScalingFactor", MaxAccelerationScalingFactor, 0.01);
    pn.param<double>("MaxVelocityScalingFactor", MaxVelocityScalingFactor, 0.05);

    ROS_INFO("Waiting for the moveit action server to come up");
    moveit::planning_interface::MoveGroup group("arm");
    group_ptr=&group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface_ptr=&planning_scene_interface;


    // group.allowReplanning(true);
    group.setMaxVelocityScalingFactor(MaxVelocityScalingFactor);
    group.setMaxAccelerationScalingFactor(MaxAccelerationScalingFactor);
    // group.setPlannerId("PRMkConfigDefault");
    group.setPlanningTime(2.0);
    group.setNumPlanningAttempts(200);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");


    ros::Subscriber pick_sub = n.subscribe("pick_go", 1, pick_go_cb);


    pick_pub=n.advertise<geometry_msgs::PoseStamped>("pick_moveit_goal", 10);
    head_object_pub=n.advertise<geometry_msgs::PoseStamped>("head_objects_in_base_frame", 10);
    arm_object_pub=n.advertise<geometry_msgs::PoseStamped>("arm_objects_in_base_frame", 10);
    pub_controller_command = n.advertise<trajectory_msgs::JointTrajectory>("/pan_tilt_trajectory_controller/command", 2);


    tf::TransformListener listener;
    listener_ptr=&listener;

    message_filters::Subscriber<geometry_msgs::PoseStamped> head_point_sub_;
    head_point_sub_.subscribe(n, head_topic, 10);
    head_tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(head_point_sub_, listener, "base_footprint", 10);
    head_tf_filter_->registerCallback( boost::bind(head_msgCallback, _1) );

    message_filters::Subscriber<geometry_msgs::PoseStamped> arm_point_sub_;
    arm_point_sub_.subscribe(n, arm_topic, 10);
    arm_tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(arm_point_sub_, listener, "base_footprint", 10);
    arm_tf_filter_->registerCallback( boost::bind(arm_msgCallback, _1) );



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
    table_primitive.dimensions[2] = 0.04;
    table_collision_object.primitives.push_back(table_primitive);
    table_collision_object.operation = table_collision_object.ADD;
    col_objects.push_back(table_collision_object);

    moveit_msgs::CollisionObject can_collision_object;
    can_collision_object.header.frame_id = "base_footprint";
    can_collision_object.id = "can";
    shape_msgs::SolidPrimitive object_primitive;
    object_primitive.type = object_primitive.CYLINDER;
    object_primitive.dimensions.resize(2);
    object_primitive.dimensions[0] = 0.15;
    object_primitive.dimensions[1] = 0.02;
    can_collision_object.primitives.push_back(object_primitive);
    can_collision_object.operation = can_collision_object.ADD;
    col_objects.push_back(can_collision_object);

    // addStaticObjects(planning_scene_interface);
    //setStartConstraints(group);
    group.setWorkspace(0.0,-2.0,0.05,2.0,2.0,2.0);

    apply_floor_constraints();
    spinner.start();



    ROS_INFO("Looking down...");
    look_down();
    //    ros::Duration(5.0);
    //    if (plan_arm("pre_grasp2")) {
    //        ROS_INFO("Arm planning is done, moving arm up..");
    //        if (group_ptr->move()) {
    //            ROS_INFO("Arm is up");
    //        }
    //    }




    ready=true;
    ROS_INFO("Ready!");

    ros::spin();
    return 0;
}

void setStartConstraints(moveit::planning_interface::MoveGroup &group) {
    moveit_msgs::Constraints constraints;
    moveit_msgs::PositionConstraint positionConstraint;
    positionConstraint.header.frame_id = "base_footprint";
    positionConstraint.link_name = "gripper_link";
    positionConstraint.weight = 1.0;

    shape_msgs::SolidPrimitive constraintsPrimitive;
    constraintsPrimitive.type = shape_msgs::SolidPrimitive::BOX;
    constraintsPrimitive.dimensions.resize(3);
    constraintsPrimitive.dimensions[0] = 1.2;
    constraintsPrimitive.dimensions[1] = 1.2;
    constraintsPrimitive.dimensions[2] = 1.2;
    positionConstraint.constraint_region.primitives.push_back(constraintsPrimitive);

    geometry_msgs::Pose primitivePos;
    primitivePos.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    primitivePos.position.x = 0;
    primitivePos.position.y = 0;
    primitivePos.position.z = 0.5;
    positionConstraint.constraint_region.primitive_poses.push_back(primitivePos);

    constraints.position_constraints.push_back(positionConstraint);
    group.setPathConstraints(constraints);
}

void addStaticObjects(const moveit::planning_interface::PlanningSceneInterface &planning_scene_interface) {
    moveit_msgs::CollisionObject floorObject;
    floorObject.header.frame_id = "base_footprint";
    floorObject.id = "floor";

    shape_msgs::SolidPrimitive floorPrimitive;
    floorPrimitive.type = shape_msgs::SolidPrimitive::BOX;
    floorPrimitive.dimensions.resize(3);
    floorPrimitive.dimensions[0] = 1.5;
    floorPrimitive.dimensions[1] = 1.5;
    floorPrimitive.dimensions[2] = 0.02;
    floorObject.primitives.push_back(floorPrimitive);

    geometry_msgs::Pose floorPos;
    floorPos.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
    floorPos.position.x = 0;
    floorPos.position.y = 0;
    floorPos.position.z = -0.015;
    floorObject.primitive_poses.push_back(floorPos);

    floorObject.operation = moveit_msgs::CollisionObject::ADD;

    staticObjects.push_back(floorObject);

    planning_scene_interface.addCollisionObjects(staticObjects);
}

