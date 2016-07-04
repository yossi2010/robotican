


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <moveit/move_group_interface/move_group.h>

typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


move_base_msgs::MoveBaseGoal move_to_table();
move_base_msgs::MoveBaseGoal move_to_button();
move_base_msgs::MoveBaseGoal get_pre_pick_pose();
void go(tf::Transform  dest);
bool gripper_cmd(double gap,double effort);
bool arm_cmd(geometry_msgs::PoseStamped target_pose1);
geometry_msgs::PoseStamped move_to_object();
geometry_msgs::PoseStamped lift_arm();
bool base_cmd(move_base_msgs::MoveBaseGoal goal);
void pick_go_cb(std_msgs::Empty);
void button_go_cb(std_msgs::Empty);
void look_down();

GripperClient *gripperClient_ptr;
MoveBaseClient *moveBaseClient_ptr;
moveit::planning_interface::MoveGroup *moveit_ptr;
tf::TransformListener *listener_ptr;

tf::StampedTransform base_obj_transform;
 std::string object_frame,depth_camera_frame;

bool have_goal=false;
//bool obj_tf_ok=false;
int image_w=0,image_h=0;
bool moving=false;



ros::Publisher planning_scene_diff_publisher;
ros::Publisher goal_pub;
ros::Publisher pan_tilt_pub;

double base_distance_from_object=0.55;
double wrist_distance_from_object=0.06;
geometry_msgs::PoseStamped moveit_goal;





move_base_msgs::MoveBaseGoal move_to_button(){

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x=-9.50;
    goal.target_pose.pose.position.y=4.108188;
    goal.target_pose.pose.position.z=0;
    goal.target_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,M_PI );
    return goal;
}
move_base_msgs::MoveBaseGoal move_to_table(){

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x=-9.0;
    goal.target_pose.pose.position.y=7.0;
    goal.target_pose.pose.position.z=0;
    goal.target_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,M_PI );
    return goal;
}



geometry_msgs::PoseStamped lift_arm(){
    geometry_msgs::PoseStamped target_pose1;

    target_pose1.header.frame_id="base_link";
    target_pose1.header.stamp=ros::Time::now();
    target_pose1.pose.position.x = 0.4;
    target_pose1.pose.position.y =  0.0;
    target_pose1.pose.position.z =  0.845;
    target_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/2.0,0 );
    return target_pose1;
}

move_base_msgs::MoveBaseGoal move_away() {

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x=-5.47;
    goal.target_pose.pose.position.y=6.62;
    goal.target_pose.pose.position.z=0;
    goal.target_pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,M_PI/2.0 );

    return goal;
}


move_base_msgs::MoveBaseGoal get_pre_pick_pose() {
    tf::Transform dest_transform;
    move_base_msgs::MoveBaseGoal goal;
    try{

        tf::StampedTransform transform_obj,transform_base;
        listener_ptr->lookupTransform("map", object_frame, ros::Time(0), transform_obj);

        listener_ptr->lookupTransform("map", "base_link", ros::Time(0), transform_base);
        tf::Vector3 v_obj =transform_obj.getOrigin();
        tf::Vector3 v_base =transform_base.getOrigin();

        tf::Vector3 v=v_obj-v_base;
        double yaw=atan2(v.y(),v.x());
        double away=base_distance_from_object/sqrt(v.x()*v.x()+v.y()*v.y());
        tf::Vector3 dest=v_base+v*(1-away);
        dest.setZ(0);
        dest_transform.setOrigin( dest );
        tf::Quaternion q;
        q.setRPY(0.0, 0, yaw);
        dest_transform.setRotation(q);
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

void look_down() {
    std_msgs::Float64MultiArray multiArray;
    multiArray.data.push_back(0.0); // pan
    multiArray.data.push_back(0.4); // tilt
    pan_tilt_pub.publish(multiArray);
}

void button_go_cb(std_msgs::Empty) {

    //navigate to the button


    if (base_cmd(move_to_button())) {

        ROS_INFO("Reached button");

        if (base_cmd(move_to_table())) {
            ROS_INFO("Reached table, asking for coke...");

            ros::param::set("/move_base/TrajectoryPlannerROS/max_vel_x", 0.1);

            char sys_cmd[]="espeak -s 150 -v en-uk 'May I have a coke please?'";
            FILE *process=popen(sys_cmd,"r");
            pclose(process);
            ros::Duration w(5);
            w.sleep();
            char sys_cmd1[]="espeak -s 150 -v female3 'Here you go'; rosrun gazebo_ros spawn_model -database coke_can_slim -sdf -model coke_can_slim -y 7.211008 -x -10.722695 -z 0.736 -Y -0.475620";
            FILE *process1=popen(sys_cmd1,"r");
            pclose(process1);
            ros::Duration w1(5);
            w1.sleep(); //wait for detection

            pick_go_cb(std_msgs::Empty());
        }
    }
}

void pick_go_cb(std_msgs::Empty) {

    if (!moving) moving=true;
     ROS_INFO("Looking down...");
 look_down();

 /*char sys_cmd1[]="espeak -s 150 -v female3 'Here you go'; rosrun gazebo_ros spawn_model -database coke_can_slim -sdf -model coke_can_slim -y 7.211008 -x -10.722695 -z 0.736 -Y -0.475620";
    FILE *process1=popen(sys_cmd1,"r");
    pclose(process1)
    ros::Duration w1(5);
    w1.sleep();
;*/
    if (base_cmd(get_pre_pick_pose())) {
        ROS_INFO("Reached pre-picking position, openning gripper...");
        if(gripper_cmd(0.14,0.0)) {
            ROS_INFO("Gripper is oppend, planning for pre-grasping..");
            ros::Duration w2(5);
            w2.sleep(); //wait for re-detection
            if (arm_cmd(move_to_object())) {
                ROS_INFO("Arm planning is done, moving arm..");
                if(moveit_ptr->move()) {
                    ROS_INFO("Ready to grasp");
                    if(gripper_cmd(0.01,0.0)) {
                        ROS_INFO("Grasping is done");
                        ros::Duration w(5);
                        w.sleep(); //wait for attach
                        ROS_INFO("Lifting object...");
                        if (arm_cmd(lift_arm())) {
                            ROS_INFO("Arm planning is done, moving arm..");
                            if (moveit_ptr->move()) {
                                ROS_INFO("Arm is up");
                                if (base_cmd(move_away())) {
                                    ROS_INFO("All done!!!");
                                }

                            }
                        }
                    }
                }
            }
        }
    }
}
geometry_msgs::PoseStamped move_to_object(){
    geometry_msgs::PoseStamped target_pose1;

    tf::StampedTransform transform_base_obj=base_obj_transform;


    tf::Vector3 v= transform_base_obj.getOrigin();

    double 	yaw=atan2(v.y(),v.x());

    target_pose1.header.frame_id="base_link";
    target_pose1.header.stamp=ros::Time::now();

    float away=wrist_distance_from_object/sqrt(v.x()*v.x()+v.y()*v.y());
    tf::Vector3 dest=v*(1-away);

    target_pose1.pose.position.x = dest.x();
    target_pose1.pose.position.y =  dest.y();
    target_pose1.pose.position.z =  v.z();
    target_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(-yaw,M_PI/2.0,0 );

    return target_pose1;

}

bool arm_cmd( geometry_msgs::PoseStamped target_pose1) {


    moveit_ptr->setStartStateToCurrentState();

    if (!have_goal) have_goal=true;

    moveit::planning_interface::MoveGroup::Plan my_plan;
    double d[]={0, 0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
    double z=target_pose1.pose.position.z;
    for (int i=0;i<sizeof(d)/sizeof(double);i++) {
        target_pose1.pose.position.z=z+d[i];
        goal_pub.publish(target_pose1);
        moveit_goal=target_pose1;
        moveit_ptr->setPoseTarget(target_pose1);
        bool success = moveit_ptr->plan(my_plan);
        ROS_INFO("Moveit plan %s",success?"SUCCESS":"FAILED");
        if (success) return true;
    }
    return false;
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


int main(int argc, char **argv) {

    ros::init(argc, argv, "demo_pick_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle n;

tf::TransformBroadcaster br;
    ROS_INFO("Hello");
  n.param<double>("base_distance_from_object", base_distance_from_object, 0.55);
   n.param<double>("wrist_distance_from_object", wrist_distance_from_object, 0.06);
    n.param<std::string>("object_frame", object_frame, "object_frame");
    n.param<std::string>("depth_camera_frame", depth_camera_frame, "kinect2_depth_optical_frame");

    GripperClient gripperClient("/gripper_controller/gripper_cmd", true);
    //wait for the gripper action server to come up
    while (!gripperClient.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the /gripper_controller/gripper_cmd action server to come up");
    }

    gripperClient_ptr=&gripperClient;

    MoveBaseClient moveBaseClient("move_base", true);
    //wait for the action server to come up
    while(!moveBaseClient.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    moveBaseClient_ptr=&moveBaseClient;

    ROS_INFO("Waiting for the moveit action server to come up");
    moveit::planning_interface::MoveGroup group("arm");

    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    group.allowReplanning(true);
    group.setPlanningTime(5.0);
    group.setNumPlanningAttempts(5);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setMaxAccelerationScalingFactor(0.01);
    group.setMaxVelocityScalingFactor(0.01);

    moveit_ptr=&group;

    ros::Subscriber pick_sub = n.subscribe("pick_go", 1, pick_go_cb);
    ros::Subscriber button_sub = n.subscribe("button_go",1,button_go_cb);

    goal_pub=n.advertise<geometry_msgs::PoseStamped>("pick_moveit_goal", 2, true);
    pan_tilt_pub = n.advertise<std_msgs::Float64MultiArray>("pan_tilt_controller/command", 10);


    tf::TransformListener listener;
    listener_ptr=&listener;

    ros::Rate r(50); // 50 hz

    ROS_INFO("Ready!");

    tf::Quaternion q;
    q.setRPY(0.0, 0, 0);


    while (ros::ok())
    {

        try{
            listener_ptr->lookupTransform("base_link", object_frame, ros::Time(0), base_obj_transform);
        }
        catch (tf::TransformException ex){
           //  ROS_ERROR("%s",ex.what());
        }
        // }
        if (have_goal) {
            tf::StampedTransform wr_goal;
            tf::Quaternion q;
            tf::quaternionMsgToTF(moveit_goal.pose.orientation,q);
            wr_goal.setOrigin(tf::Vector3(moveit_goal.pose.position.x,moveit_goal.pose.position.y,moveit_goal.pose.position.z));
            wr_goal.setRotation((q));
            wr_goal.stamp_=ros::Time::now();
            wr_goal.child_frame_id_="moveit_goal";
            wr_goal.frame_id_="base_link";
            br.sendTransform(wr_goal);

        }


        r.sleep();
    }

    return 0;
}

