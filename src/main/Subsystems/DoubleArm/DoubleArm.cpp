// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <math.h>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>
#include "DoubleArm.h"
#include <iostream>

DoubleArm::DoubleArm() {
    planner.SetTargetCoord(GetEndpointCoord(), GetEndpointCoord());

    frc::SmartDashboard::PutData(&plotter);
}
/**
 * Will be called periodically whenever the CommandScheduler runs.
 */
void DoubleArm::Periodic() {
    frc::SmartDashboard::PutNumber("Lower Angle", getLowerAngle());
    frc::SmartDashboard::PutNumber("Upper Angle", getUpperAngle());

    std::optional<DoubleArmState> desiredState = planner.CalculateCurrentTargetState();
    if (desiredState.has_value()) {
        DoubleArmState targetState = desiredState.value();
        frc::SmartDashboard::PutNumber("DoubleArm/1.LowerAngleTarget", targetState.lowerAngle.Degrees().value());
        frc::SmartDashboard::PutNumber("DoubleArm/2.UpperAngleTarget", targetState.upperAngle.Degrees().value());

        auto targetCoord = kinematics.GetEndpointCoord(targetState);
        plotter.SetRobotPose({ targetCoord + frc::Translation2d{4_m, 4_m}, 0_deg });
        frc::SmartDashboard::PutNumber("DoubleArm/DesiredX", targetCoord.X().value());
        frc::SmartDashboard::PutNumber("DoubleArm/DesiredY", targetCoord.Y().value());
    }

    frc::Translation2d currentCoord = GetEndpointCoord();
    frc::SmartDashboard::PutNumber("DoubleArm/X", currentCoord.X().value());
    frc::SmartDashboard::PutNumber("DoubleArm/Y", currentCoord.Y().value());
    frc::SmartDashboard::PutNumber("DoubleArm/3.PlannerFinished", planner.IsFinished());

}

DoubleArmState DoubleArm::GetCurrentState() {
    DoubleArmState state;
    state.lowerAngle = 0_deg;
    state.upperAngle = 0_deg;
    return state;
}

frc::Translation2d DoubleArm::GetEndpointCoord() {
    return kinematics.GetEndpointCoord(GetCurrentState());
}

void DoubleArm::SetTargetCoord(frc::Translation2d targetCoord) {
    planner.SetTargetCoord(targetCoord, GetEndpointCoord());
}

void DoubleArm::motorConfiguration() {
    /* Lower Motors */
    lowerRight.ConfigFactoryDefault();
    lowerRightInverted.ConfigFactoryDefault();
    lowerLeft.ConfigFactoryDefault();
    lowerLeftInverted.ConfigFactoryDefault();

    lowerRightInverted.SetInverted(true);
    lowerLeftInverted.SetInverted(true);

    lowerRightInverted.Follow(lowerRight);
    lowerLeftInverted.Follow(lowerLeft);

    /* Upper Motors */
    upperRight.ConfigFactoryDefault();
    upperLeft.ConfigFactoryDefault();

    upperLeft.SetInverted(true);
    upperLeft.Follow(upperRight);
}

double DoubleArm::getLowerAngle() {
    return dutyCycleToDegrees(lowerEncoder.GetAbsolutePosition());
}

double DoubleArm::getUpperAngle() {
    return dutyCycleToDegrees(upperEncoder.GetAbsolutePosition());
}

double DoubleArm::dutyCycleToDegrees(double dutyCycleUnits) {
    return dutyCycleUnits * 360;
}

double DoubleArm::dutyCycleToCTREUnits(double dutyCyclePos) {
    return dutyCyclePos * 4096;
}