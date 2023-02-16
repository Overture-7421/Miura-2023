// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <math.h>
#include <cmath>

class DoubleArm: public frc2::SubsystemBase {
public:
    DoubleArm();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    void SetArmPosition(double Angle1, double Angle2, double TargetX, double TargetY);

private:
};
