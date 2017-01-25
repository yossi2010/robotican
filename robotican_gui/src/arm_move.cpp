#include "arm_move.h"

ArmMove::ArmMove() : _group("arm")
{
    _group.setPlannerId("PRMstarkConfigDefault");
    _group.setPlanningTime(5.0);
    _group.setNumPlanningAttempts(2);
    _group.setMaxVelocityScalingFactor(0.1);
    _isSuccess = false;
}

bool ArmMove::plan(std::string targetName)
{
    _isSuccess = false;
    _group.setNamedTarget(targetName);
    if  (_group.plan(_drivingModePlan))
        _isSuccess = true;
    return _isSuccess;
}

void ArmMove::move()
{
    if (_isSuccess)
        _group.move();
}