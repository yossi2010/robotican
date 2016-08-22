//
// Created by sub on 21/08/16.
//

#ifndef ROBOTICAN_GUI_DRIVEMODE_H
#define ROBOTICAN_GUI_DRIVEMODE_H

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

class DriveMode {
private:
    moveit::planning_interface::MoveGroup::Plan _drivingModePlan;
    moveit::planning_interface::MoveGroup _group;

public:
    DriveMode();
    bool moveArm();

};


#endif //ROBOTICAN_GUI_DRIVEMODE_H
