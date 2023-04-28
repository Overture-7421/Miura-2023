// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetArmCoordinate.h"

SetArmCoordinate::SetArmCoordinate(DoubleArm* doubleArm, DoubleArmState targetState) {
    this->doubleArm = doubleArm;
    this->targetState = targetState;

    AddRequirements(doubleArm);
}

// Called when the command is initially scheduled.
void SetArmCoordinate::Initialize() {
    doubleArm->SetTargetCoord(targetState);
}

// Called repeatedly when this Command is scheduled to run
void SetArmCoordinate::Execute() {}

// Called once the command ends or is interrupted.
void SetArmCoordinate::End(bool interrupted) {}

// Returns true when the command should end.
bool SetArmCoordinate::IsFinished() {
    DoubleArmState remainingDistance = doubleArm->IsAtTarget();
    return remainingDistance.lowerAngle.Degrees() < 3_deg && remainingDistance.upperAngle.Degrees() < 3_deg;
}