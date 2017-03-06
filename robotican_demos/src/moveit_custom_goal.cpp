//
// Created by tom on 20/04/16.
//

//rosrun robotican_demos moveit_custom_goal 0.50 0.0 0.95 n //params: x, y, z, y/n (determine if trying offsets around goal is enabled)


#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>

using namespace std;

robot_state::GroupStateValidityCallbackFn state_validity_callback_fn_;
robot_state::RobotStatePtr *robot_state_ptr;
planning_scene::PlanningScenePtr *planning_scene_ptr;

const robot_model::JointModelGroup* joint_model_group;

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
    bool found_ik = (*robot_state_ptr)->setFromIK(joint_model_group, pose.pose, 100,30.0, state_validity_callback_fn_);
    std::printf("IK %d: [%f , %f , %f] [%f , %f , %f , %f]\n",found_ik,pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
    return found_ik;
}


int main(int argc, char* argv[]) {
    if (argc != 5)
    {
        ROS_ERROR("X, Y, Z, y/n arguments needed");
        return -1;
    }

    ros::init(argc, argv,"moveit_group_goal");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;

    moveit::planning_interface::MoveGroup group("arm");

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


    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.5);
    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(500);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");
    group.setGoalTolerance(0.1);

    group.setStartStateToCurrentState();


    state_validity_callback_fn_ = boost::bind(&isIKSolutionCollisionFree, _1, _2, _3);

    // ROS_INFO("End effector reference frame: %s", group.getEndEffectorLink().c_str());

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id="base_footprint";
    target_pose.header.stamp=ros::Time::now()+ros::Duration(10.0);
    target_pose.pose.position.x = atof(argv[1]);
    target_pose.pose.position.y = atof(argv[2]);
    target_pose.pose.position.z = atof(argv[3]);
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0.0,0.0); //horizontal

    if (checkIK(target_pose)) {
        ROS_ERROR("Valid IK");
        group.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroup::Plan my_plan;
        bool success = group.plan(my_plan);
        ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
        if(success) {
            ROS_INFO("Moving...");
            group.move();
        }
    }
    else if(*argv[4] == 'n'){
        ROS_ERROR("Invalid IK");
    }
    else if (*argv[4] == 'y')
    {
        ROS_INFO("Invalid IK, Trying same goal + offset");
        moveit::planning_interface::MoveGroup::Plan my_plan;
        double dz[]={0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
        double dy[]={0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
        double dx[]={0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
        double dY[]={0.01, -0.01 ,0.02, -0.02,0.03, -0.03};
        double z=target_pose.pose.position.z;
        double x=target_pose.pose.position.x;
        double y=target_pose.pose.position.y;
        tf::Quaternion q( target_pose.pose.orientation.x,  target_pose.pose.orientation.y,  target_pose.pose.orientation.z, target_pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        for (int n=0;n<sizeof(dx)/sizeof(double);n++) {
            for (int m=0;m<sizeof(dy)/sizeof(double);m++) {

                for (int i=0;i<sizeof(dz)/sizeof(double);i++) {
                    for (int j=0;j<sizeof(dY)/sizeof(double);j++) {
                        target_pose.pose.position.z=z+dz[i];
                        target_pose.pose.position.x=x+dx[n];
                        target_pose.pose.position.y=y+dy[m];

                        target_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw+dY[j] );


                        if (checkIK(target_pose)) {
                            group.setPoseTarget(target_pose);
                            moveit::planning_interface::MoveGroup::Plan my_plan;
                            bool success = group.plan(my_plan);
                            ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
                            if(success) {
                                ROS_INFO("Moving...");
                                group.move();
                                return 0;
                            }
                        }
                        else
                        {
                            ROS_WARN("planning goal + offset number %i failed");
                        }
                    }
                }
            }
        }
    }
    sleep(5);



    return 0;
}
