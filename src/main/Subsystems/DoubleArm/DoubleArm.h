// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Translation2d.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <CTRE/Phoenix.h>

#include "DoubleArmState/DoubleArmState.h"

class DoubleArm: public frc2::SubsystemBase {
public:
    DoubleArm();
    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    DoubleArmState GetCurrentState();
    void SetTargetCoord(DoubleArmState targetState);
    frc::Rotation2d GetLowerAngle();
    frc::Rotation2d GetUpperAngle();


private:
    void SetFalconTargetPos(DoubleArmState desiredState);
    void ConfigureMotors();
    void ConfigureSensors();

    double ConvertAngleToUpperFalconPos(frc::Rotation2d angle);
    double ConvertAngleToLowerFalconPos(frc::Rotation2d angle);

    DoubleArmState targetState = { 101_deg, -71_deg };

    const double FALCON_CODES_PER_REV = 2048;
    const double LOWER_GEARBOX_REDUCTION = 96;
    const double UPPER_GEARBOX_REDUCTION = 95.2;

    const double CODES_PER_LOWER_ROTATION = LOWER_GEARBOX_REDUCTION * FALCON_CODES_PER_REV;
    const double CODES_PER_UPPER_ROTATION = UPPER_GEARBOX_REDUCTION * FALCON_CODES_PER_REV;

    /* Lower Motors */
    TalonFX lowerRight{ 9 };
    TalonFX lowerRight2{ 10 };
    TalonFX lowerLeft{ 11 };
    TalonFX lowerLeft2{ 12 };

    /* Upper Motors */
    TalonFX upperRight{ 14 };
    TalonFX upperLeft{ 13 };

    /* Encoders */
    frc::DutyCycleEncoder lowerEncoder{ 1 };
    frc::DutyCycleEncoder upperEncoder{ 0 };

    double lowerFeedForward = 0.03;
    double upperFeedForward = 0.04;

    double lowerKP = 0.05; //Misma variación desde 0.08
    double upperKP = 0.06;
};
