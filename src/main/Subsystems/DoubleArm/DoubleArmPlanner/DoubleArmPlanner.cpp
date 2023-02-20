// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DoubleArmPlanner.h"
#include <frc/Timer.h>

DoubleArmPlanner::DoubleArmPlanner(PlannerProfile::Constraints constraints): targetPointProfile(constraints, PlannerProfile::State{ PlannerProfile::Distance_t{0}, PlannerProfile::Velocity_t{0} }) {
    trajectoryStartTime = frc::Timer::GetFPGATimestamp();
}

void DoubleArmPlanner::SetTargetCoord(frc::Translation2d targetCoord, frc::Translation2d currentCoord) {

    double travelDistance = currentCoord.Distance(targetCoord).value();
    trajectoryAngle = (targetCoord - currentCoord).Angle();
    startingCoord = currentCoord;

    targetPointProfile = PlannerProfile(constraints, PlannerProfile::State{ PlannerProfile::Distance_t{travelDistance}, PlannerProfile::Velocity_t{0} });
    trajectoryStartTime = frc::Timer::GetFPGATimestamp();
}

DoubleArmState DoubleArmPlanner::CalculateCurrentTargetState() {
    PlannerProfile::State desiredProfileState = targetPointProfile.Calculate(frc::Timer::GetFPGATimestamp() - trajectoryStartTime);


    frc::Translation2d displacementCoord{ units::meter_t(trajectoryAngle.Cos() * desiredProfileState.position), units::meter_t(trajectoryAngle.Sin() * desiredProfileState.position) };

    frc::Translation2d targetCoord = startingCoord + displacementCoord;

    return kinematics.GetStateForTargetCoord(targetCoord);
}

bool DoubleArmPlanner::IsFinished() {
    return targetPointProfile.IsFinished(frc::Timer::GetFPGATimestamp() - trajectoryStartTime);
}