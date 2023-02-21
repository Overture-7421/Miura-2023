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

#include "DoubleArmKinematics/DoubleArmKinematics.h"
#include "DoubleArmPlanner/DoubleArmPlanner.h"



class DoubleArm: public frc2::SubsystemBase {
public:
    DoubleArm();
    void motorConfiguration();
    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    DoubleArmState GetCurrentState();
    frc::Translation2d GetEndpointCoord();
    void SetTargetCoord(frc::Translation2d targetCoord);
    double getLowerAngle();
    double getUpperAngle();
    double dutyCycleToDegrees(double dutyCycleUnits);
    double dutyCycleToCTREUnits(double dutyCyclePos);

private:
    DoubleArmKinematics kinematics{ 0.8382, 0.8382 };
    DoubleArmPlanner planner{ {3.0_mps, 5.0_mps_sq} , kinematics }; // Constraints are meters per second, max accel of meters per second squared
    frc::Field2d plotter;

    /* Lower Motors */
    WPI_TalonFX lowerRight{ 9 };
    WPI_TalonFX lowerRightInverted{ 10 };
    WPI_TalonFX lowerLeft{ 11 };
    WPI_TalonFX lowerLeftInverted{ 12 };

    /* Upper Motors */
    WPI_TalonFX upperRight{ 13 };
    WPI_TalonFX upperLeft{ 14 };

    /* Encoders */
    frc::DutyCycleEncoder lowerEncoder{ 0 };
    frc::DutyCycleEncoder upperEncoder{ 1 };
};
