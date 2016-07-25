


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <moveit/move_group_interface/move_group.h>

typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;



void go(tf::Transform  dest);
bool gripper_cmd(double gap,double effort);
bool arm_cmd(geometry_msgs::PoseStamped target_pose1);
geometry_msgs::PoseStamped move_to_object();


geometry_msgs::PoseStamped lift_arm();


void pick_go_cb(std_msgs::Empty);
void button_go_cb(std_msgs::Empty);
void look_down();

GripperClient *gripperClient_ptr;

moveit::planning_interface::MoveGroup *moveit_ptr;
tf::TransformListener *listener_ptr;

tf::StampedTransform base_obj_transform;
std::string object_frame;

geometry_msgs::PoseStamped pick_pose;
double pick_yaw=0;
bool have_goal=false;
bool moving=false;



ros::Publisher planning_scene_diff_publisher;
ros::Publisher goal_pub;
ros::Publisher pan_tilt_pub;

double wrist_distance_from_object=0.10;
geometry_msgs::PoseStamped moveit_goal;



geometry_msgs::PoseStamped lift_arm(){
    geometry_msgs::PoseStamped target_pose1;
    target_pose1.header.frame_id="base_link";
    target_pose1.header.stamp=ros::Time::now();
    target_pose1.pose.position.x = 0.4;
    target_pose1.pose.position.y =  0.0;
    target_pose1.pose.position.z =  0.85;
    target_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/2.0,0 );
    return target_pose1;
}


void look_down() {
    std_msgs::Float64MultiArray multiArray;
    multiArray.data.push_back(0.0); // pan
    multiArray.data.push_back(0.4); // tilt
    pan_tilt_pub.publish(multiArray);
}


geometry_msgs::PoseStamped move_to_object(){
    geometry_msgs::PoseStamped target_pose1;

    tf::StampedTransform transform_base_obj=base_obj_transform;


    tf::Vector3 v= transform_base_obj.getOrigin();

    pick_yaw=atan2(v.y(),v.x());

    target_pose1.header.frame_id="base_link";
    target_pose1.header.stamp=ros::Time::now();

    float away=wrist_distance_from_object/sqrt(v.x()*v.x()+v.y()*v.y());
    tf::Vector3 dest=v*(1-away);

    target_pose1.pose.position.x = dest.x();
    target_pose1.pose.position.y =  dest.y();
    target_pose1.pose.position.z =  v.z();
    target_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(-pick_yaw,M_PI/2.0,0 );

    return target_pose1;

}

void pick_go_cb(std_msgs::Empty) {

    if (!moving) moving=true;
    ROS_INFO("Openning gripper...");
    if(gripper_cmd(0.14,0.0)) {
        ROS_INFO("Gripper is oppend, planning for pre-grasping..");
        ros::Duration w2(5);
        w2.sleep(); //wait for re-detection
        pick_pose=move_to_object();
        if (arm_cmd(pick_pose)) {
            ROS_INFO("Arm planning is done, moving arm..");
            if(moveit_ptr->move()) {
                ROS_INFO("Ready to grasp");
                if(gripper_cmd(0.01,0.0)) {
                    ROS_INFO("Grasping is done");
                    ros::Duration w(8);
                    w.sleep(); //wait for attach
                    ROS_INFO("Lifting object...");
                    if (arm_cmd(lift_arm())) {
                        ROS_INFO("Arm planning is done, moving arm up..");
                        if (moveit_ptr->move()) {
                            ROS_INFO("Arm is up, placing on table...");
                            pick_pose.pose.position.z=pick_pose.pose.position.z+0.01;
                            pick_pose.pose.position.y=pick_pose.pose.position.y+0.1;
                            if (arm_cmd(pick_pose)) {
                                ROS_INFO("Arm planning is done, moving arm..");
                                if(moveit_ptr->move()) {
                                    ROS_INFO("Openning gripper...");
                                    if(gripper_cmd(0.14,0.0)) {
                                        ROS_INFO("Done!");
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

bool arm_cmd( geometry_msgs::PoseStamped target_pose1) {


    moveit_ptr->setStartStateToCurrentState();

    if (!have_goal) have_goal=true;

    moveit::planning_interface::MoveGroup::Plan my_plan;
    double dz[]={0, 0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
       double dy[]={0, 0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
          double dx[]={0, 0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
    double dY[]={0, 0.04, -0.04 ,0.08, -0.08};
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
            target_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(-pick_yaw+dY[j],M_PI/2.0,0.0 );
            goal_pub.publish(target_pose1);
            moveit_goal=target_pose1;
           std::printf("---------> PLAN: [%f , %f , %f] [%f , %f , %f]\n",target_pose1.pose.position.x,target_pose1.pose.position.y,target_pose1.pose.position.z,-pick_yaw+dY[j],M_PI/2.0,0.0);
            moveit_ptr->setPoseTarget(target_pose1);
            bool success = moveit_ptr->plan(my_plan);
            ROS_INFO("Moveit plan %s",success?"SUCCESS":"FAILED");
            if (success) return true;
        }
    }
            }
      }
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


int main(int argc, char **argv) {

    ros::init(argc, argv, "pick_and_plce_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle n;

    tf::TransformBroadcaster br;
    ROS_INFO("Hello");

    n.param<double>("wrist_distance_from_object", wrist_distance_from_object, 0.10);
    n.param<std::string>("object_frame", object_frame, "object_frame");


    GripperClient gripperClient("/gripper_controller/gripper_cmd", true);
    //wait for the gripper action server to come up
    while (!gripperClient.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the /gripper_controller/gripper_cmd action server to come up");
    }

    gripperClient_ptr=&gripperClient;


    ROS_INFO("Waiting for the moveit action server to come up");
    moveit::planning_interface::MoveGroup group("arm");

    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

   // group.allowReplanning(true);
    // group.setPlanningTime(5.0);
     //group.setNumPlanningAttempts(5);
    group.setPlannerId("RRTConnectkConfigDefault");
    //group.setPlannerId("LBKPIECEkConfigDefault");
    group.setMaxAccelerationScalingFactor(0.1);
    group.setMaxVelocityScalingFactor(0.1);
     group.setGoalPositionTolerance(0.02);

    moveit_ptr=&group;

    ros::Subscriber pick_sub = n.subscribe("pick_go", 1, pick_go_cb);

    goal_pub=n.advertise<geometry_msgs::PoseStamped>("pick_moveit_goal", 2, true);
    pan_tilt_pub = n.advertise<std_msgs::Float64MultiArray>("/pan_tilt_controller/command", 10);


    tf::TransformListener listener;
    listener_ptr=&listener;

    ros::Rate r(50); // 50 hz



    tf::Quaternion q;
    q.setRPY(0.0, 0, 0);

    ros::Duration(3.0).sleep();
    ROS_INFO("Looking down...");
    look_down();
    if (arm_cmd(lift_arm())) {
        ROS_INFO("Arm planning is done, moving arm up..");
        if (moveit_ptr->move()) {
            ROS_INFO("Arm is up");
        }
    }
    ROS_INFO("Ready!");

    while (ros::ok())
    {


        try{
            listener_ptr->lookupTransform("base_link", object_frame, ros::Time(0), base_obj_transform);
        }
        catch (tf::TransformException ex){
              ROS_ERROR("PNP NODE: %s",ex.what());
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

