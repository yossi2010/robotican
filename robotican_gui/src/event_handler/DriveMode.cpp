//
// Created by sub on 21/08/16.
//

#include "DriveMode.h"

DriveMode::DriveMode() : _group("arm") {
    _group.setPlannerId("SBLkConfigDefault");
    _group.setPlanningTime(5.0);
    _group.setNumPlanningAttempts(2);
}

bool DriveMode::moveArm()
{
    //plan
    _group.setNamedTarget("Driving Mode");


    bool plan_success = _group.plan(_drivingModePlan);

    //execute
    if (plan_success)
    {
        _group.move();
        return true;
    }

    return false;
}