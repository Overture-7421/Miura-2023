// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <math.h>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>
#include "DoubleArm.h"
#include <iostream>

DoubleArm::DoubleArm() {
    ConfigureMotors();
    ConfigureSensors();

    frc::SmartDashboard::PutData(&plotter);
    planner.SetTargetCoord(GetEndpointCoord(), GetEndpointCoord());
}
/**
 * Will be called periodically whenever the CommandScheduler runs.
 */
void DoubleArm::Periodic() {
    auto currentState = GetCurrentState();
    frc::SmartDashboard::PutNumber("DoubleArm/CurrentLowerAngle", currentState.lowerAngle.Degrees().value());
    frc::SmartDashboard::PutNumber("DoubleArm/CurrentUpperAngle", currentState.upperAngle.Degrees().value());

    frc::SmartDashboard::PutNumber("DoubleArm/EncoderLowerRaw", lowerEncoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("DoubleArm/EncoderUpperRaw", upperEncoder.GetAbsolutePosition());

    std::optional<DoubleArmState> desiredState = planner.CalculateCurrentTargetState();
    if (desiredState.has_value()) {
        DoubleArmState targetState = desiredState.value();
        frc::SmartDashboard::PutNumber("DoubleArm/TargetLowerAngle", targetState.lowerAngle.Degrees().value());
        frc::SmartDashboard::PutNumber("DoubleArm/TargetUpperAngle", targetState.upperAngle.Degrees().value());

        auto targetCoord = kinematics.GetEndpointCoord(targetState);
        plotter.SetRobotPose({ targetCoord + frc::Translation2d{4_m, 4_m}, 0_deg });
        frc::SmartDashboard::PutNumber("DoubleArm/DesiredX", targetCoord.X().value());
        frc::SmartDashboard::PutNumber("DoubleArm/DesiredY", targetCoord.Y().value());

        //   SetFalconTargetPos(targetState);
    }

    frc::Translation2d currentCoord = kinematics.GetEndpointCoord(currentState);
    frc::SmartDashboard::PutNumber("DoubleArm/X", currentCoord.X().value());
    frc::SmartDashboard::PutNumber("DoubleArm/Y", currentCoord.Y().value());
    frc::SmartDashboard::PutNumber("DoubleArm/PlannerFinished", planner.IsFinished());

}

DoubleArmState DoubleArm::GetCurrentState() {
    DoubleArmState state;
    state.lowerAngle = GetLowerAngle();
    state.upperAngle = GetUpperAngle();
    return state;
}

frc::Translation2d DoubleArm::GetEndpointCoord() {
    return kinematics.GetEndpointCoord(GetCurrentState());
}

void DoubleArm::SetTargetCoord(frc::Translation2d targetCoord) {
    planner.SetTargetCoord(targetCoord, GetEndpointCoord());
}

void DoubleArm::SetFalconTargetPos(DoubleArmState desiredState) {
    lowerRight.Set(ControlMode::Position, ConvertAngleToLowerFalconPos(desiredState.lowerAngle), DemandType_ArbitraryFeedForward, 0.0 * desiredState.lowerAngle.Cos());
    upperRight.Set(ControlMode::Position, ConvertAngleToUpperFalconPos(desiredState.upperAngle), DemandType_ArbitraryFeedForward, 0.0 * desiredState.upperAngle.Cos());
}

void DoubleArm::ConfigureMotors() {
    TalonFXConfiguration baseConfig;
    baseConfig.voltageCompSaturation = 12.0;
    baseConfig.supplyCurrLimit = SupplyCurrentLimitConfiguration(true, 20, 30, 100);

    /* Lower Motors */
    lowerRight.ConfigAllSettings(baseConfig);
    lowerRight.EnableVoltageCompensation(true);

    lowerRight2.ConfigAllSettings(baseConfig);
    lowerRight2.EnableVoltageCompensation(true);

    lowerLeft.ConfigAllSettings(baseConfig);
    lowerLeft.EnableVoltageCompensation(true);

    lowerLeft2.ConfigAllSettings(baseConfig);
    lowerLeft2.EnableVoltageCompensation(true);

    lowerRight2.Follow(lowerRight);
    lowerLeft2.Follow(lowerRight);
    lowerLeft.Follow(lowerRight);

    lowerRight2.SetInverted(TalonFXInvertType::FollowMaster);
    lowerLeft.SetInverted(TalonFXInvertType::OpposeMaster);
    lowerLeft2.SetInverted(TalonFXInvertType::OpposeMaster);

    lowerRight.SelectProfileSlot(0, 0);
    lowerRight.Config_kP(0, 0);
    lowerRight.Config_kI(0, 0);
    lowerRight.Config_kD(0, 0);

    /* Upper Motors */
    upperRight.ConfigAllSettings(baseConfig);
    upperRight.EnableVoltageCompensation(true);

    upperLeft.ConfigAllSettings(baseConfig);
    upperLeft.EnableVoltageCompensation(true);

    upperLeft.Follow(upperRight);
    upperLeft.SetInverted(TalonFXInvertType::OpposeMaster);

    upperRight.SelectProfileSlot(0, 0);
    upperRight.Config_kP(0, 0);
    upperRight.Config_kI(0, 0);
    upperRight.Config_kD(0, 0);
}

void DoubleArm::ConfigureSensors() {
    lowerEncoder.SetPositionOffset(0); //TODO: Get offsets so angle is relative to X axis when calling GetLowerAngle and GetUpperAngle.
    upperEncoder.SetPositionOffset(0);

    lowerRight.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative);
    upperRight.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative);

    lowerRight.SetSelectedSensorPosition(ConvertAngleToLowerFalconPos(GetLowerAngle()));
    upperRight.SetSelectedSensorPosition(ConvertAngleToUpperFalconPos(GetUpperAngle()));

    lowerRight.SetSensorPhase(false); // TODO: Check sensors phase
    upperRight.SetSensorPhase(false);
}

frc::Rotation2d DoubleArm::GetLowerAngle() {
    return units::degree_t(lowerEncoder.GetAbsolutePosition() * 360.0);
}

frc::Rotation2d DoubleArm::GetUpperAngle() {
    return frc::Rotation2d(units::degree_t(upperEncoder.GetAbsolutePosition() * 360.0)) + GetLowerAngle();
}

double DoubleArm::ConvertAngleToLowerFalconPos(frc::Rotation2d angle) {
    return angle.Degrees().value() / 360 * CODES_PER_LOWER_ROTATION;
}
double DoubleArm::ConvertAngleToUpperFalconPos(frc::Rotation2d angle) {
    return angle.Degrees().value() / 360 * CODES_PER_UPPER_ROTATION;
}