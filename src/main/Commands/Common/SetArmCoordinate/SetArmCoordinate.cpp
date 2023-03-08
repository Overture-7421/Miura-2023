// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SetArmCoordinate.h"

SetArmCoordinate::SetArmCoordinate(DoubleArm* doubleArm, frc::Translation2d target, PlannerProfile::Constraints constraints) {
    this->doubleArm = doubleArm;
    this->target = target;
    this->constraints = constraints;

    AddRequirements(doubleArm);
}

// Called when the command is initially scheduled.
void SetArmCoordinate::Initialize() {
    doubleArm->SetTargetCoord(target, constraints);
}

// Called repeatedly when this Command is scheduled to run
void SetArmCoordinate::Execute() {}

// Called once the command ends or is interrupted.
void SetArmCoordinate::End(bool interrupted) {}

// Returns true when the command should end.
bool SetArmCoordinate::IsFinished() {
    return target.Distance(doubleArm->GetEndpointCoord()) < .1_m;
}