// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DoubleArm.h"
#include <math.h>
#include <cmath>

DoubleArm::DoubleArm() = default;

// This method will be called once per scheduler run
void DoubleArm::Periodic() {}
void SetArmPosition(double Angle1, double Angle2, double TargetX, double TargetY) {
    double Length1 = 33;  //inches
    double Length2 = 33;  //inches

    double TargetAngle2 = (acos((pow(TargetX, 2) + pow(TargetY, 2) - pow(Length1, 2) - pow(Length2, 2) / (2 * Length1 * Length2)))) * -1;
    double TargetAngle1 = atan2(TargetY, TargetX) + atan2((Length2 * sin(TargetAngle2)), (Length1 + Length2 * cos(TargetAngle2)));

    double FinalMovementAngle2 = TargetAngle2 - Angle2;
    double FinalMovementAngle1 = TargetAngle1 - Angle1;
}


//  acos(x) = cos^-1(x)
//  atan2(x) = tan^-1(x)
//  sin(x) = sin x
//  cos(x) = cos x