//
// Created by sub on 21/08/16.
//

#include "ArmMove.h"


ArmMove::ArmMove() : _group("arm")
{
    _targetName = "driving";
    _group.setPlannerId("SBLkConfigDefault");
    _group.setPlanningTime(5.0);
    _group.setNumPlanningAttempts(2);
    _group.setMaxVelocityScalingFactor(0.1);
    _isSuccess = false;
}

bool ArmMove::plan()
{
    _isSuccess = false;
    _group.setNamedTarget("driving");
    if  (_group.plan(_drivingModePlan))
    {
        _isSuccess = true;
    }
    return _isSuccess;
}

void ArmMove::move()
{
    if (_isSuccess)
    {
        _group.move();
    }
}