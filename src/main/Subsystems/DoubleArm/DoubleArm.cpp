// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <math.h>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include "DoubleArm.h"
#include <iostream>
#include <thread>

DoubleArm::DoubleArm() {
    ConfigureMotors();
    ConfigureSensors();

    // frc::SmartDashboard::PutData(&plotter);
    planner.SetTargetCoord(GetEndpointCoord(), GetEndpointCoord(), { 1_mps, 1_mps_sq });
}
/**
 * Will be called periodically whenever the CommandScheduler runs.
 */
void DoubleArm::Periodic() {
    //auto currentState = GetCurrentState();
    //frc::SmartDashboard::PutNumber("DoubleArm/CurrentLowerAngle", currentState.lowerAngle.Degrees().value());
    //frc::SmartDashboard::PutNumber("DoubleArm/CurrentUpperAngle", currentState.upperAngle.Degrees().value());

    frc::SmartDashboard::PutNumber("DoubleArm/EncoderLowerRaw", lowerEncoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("DoubleArm/EncoderUpperRaw", upperEncoder.GetAbsolutePosition());


    std::optional<DoubleArmState> desiredState = planner.CalculateCurrentTargetState();
    if (desiredState.has_value()) {
        targetState = desiredState.value();
        // frc::SmartDashboard::PutNumber("DoubleArm/TargetLowerAngle", targetState.lowerAngle.Degrees().value());
        // frc::SmartDashboard::PutNumber("DoubleArm/TargetUpperAngle", targetState.upperAngle.Degrees().value());

        // auto targetCoord = kinematics.GetEndpointCoord(targetState);
        // plotter.SetRobotPose({ targetCoord + frc::Translation2d{4_m, 4_m}, 0_deg });
        // frc::SmartDashboard::PutNumber("DoubleArm/DesiredX", targetCoord.X().value());
        // frc::SmartDashboard::PutNumber("DoubleArm/DesiredY", targetCoord.Y().value());

    }
    SetFalconTargetPos(targetState);

    // frc::Translation2d currentCoord = kinematics.GetEndpointCoord(currentState);
    // frc::SmartDashboard::PutNumber("DoubleArm/X", currentCoord.X().value());
    // frc::SmartDashboard::PutNumber("DoubleArm/Y", currentCoord.Y().value());
    // frc::SmartDashboard::PutNumber("DoubleArm/PlannerFinished", planner.IsFinished());

    // frc::SmartDashboard::PutNumber("AngleLowCont", ConvertAngleToLowerFalconPos(GetLowerAngle()));
    // frc::SmartDashboard::PutNumber("AngleUpCont", ConvertAngleToUpperFalconPos(GetUpperAngle()));



    // lowerRight.Set(ControlMode::Position, ConvertAngleToLowerFalconPos(desiredState.lowerAngle), DemandType_ArbitraryFeedForward, 0.03 * desiredState.lowerAngle.Cos());
    //upperRight.Set(ControlMode::Position, frc::SmartDashboard::GetNumber("4.Target", 0.0), DemandType_ArbitraryFeedForward, 0.031 * currentState.upperAngle.Cos());

    // frc::SmartDashboard::PutNumber("5.TargetLower", lowerRight.GetClosedLoopTarget());
    // frc::SmartDashboard::PutNumber("5.TargetUpper", upperRight.GetClosedLoopTarget());

    // frc::SmartDashboard::PutNumber("5.CurrentPosLower", lowerRight.GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("5.CurrentPosUpper", upperRight.GetSelectedSensorPosition());
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

void DoubleArm::SetTargetCoord(frc::Translation2d targetCoord, PlannerProfile::Constraints constraints) {
    planner.SetTargetCoord(targetCoord, GetEndpointCoord(), constraints);
}

void DoubleArm::SetFalconTargetPos(DoubleArmState desiredState) {
    lowerRight.Set(ControlMode::Position, ConvertAngleToLowerFalconPos(desiredState.lowerAngle), DemandType_ArbitraryFeedForward, 0.180 * desiredState.lowerAngle.Cos());
    upperRight.Set(ControlMode::Position, ConvertAngleToUpperFalconPos(desiredState.upperAngle), DemandType_ArbitraryFeedForward, 0.060 * desiredState.upperAngle.Cos());
}

void DoubleArm::ConfigureMotors() {
    TalonFXConfiguration baseConfig;
    baseConfig.voltageCompSaturation = 12.0;
    baseConfig.supplyCurrLimit = SupplyCurrentLimitConfiguration(true, 25, 30, 0);
    baseConfig.neutralDeadband = 0.001;

    /* Lower Motors */
    lowerRight.ConfigAllSettings(baseConfig);
    lowerRight.EnableVoltageCompensation(true);
    lowerRight.SetNeutralMode(NeutralMode::Brake);

    lowerRight2.ConfigAllSettings(baseConfig);
    lowerRight2.EnableVoltageCompensation(true);
    lowerRight2.SetNeutralMode(NeutralMode::Brake);
    lowerRight2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 255);
    lowerRight2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 255);


    lowerLeft.ConfigAllSettings(baseConfig);
    lowerLeft.EnableVoltageCompensation(true);
    lowerLeft.SetNeutralMode(NeutralMode::Brake);
    lowerLeft.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 255);
    lowerLeft.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 255);

    lowerLeft2.ConfigAllSettings(baseConfig);
    lowerLeft2.EnableVoltageCompensation(true);
    lowerLeft2.SetNeutralMode(NeutralMode::Brake);
    lowerLeft2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 255);
    lowerLeft2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 255);

    lowerRight2.Follow(lowerRight);
    lowerLeft2.Follow(lowerRight);
    lowerLeft.Follow(lowerRight);

    lowerRight2.SetInverted(TalonFXInvertType::FollowMaster);
    lowerLeft.SetInverted(TalonFXInvertType::OpposeMaster);
    lowerLeft2.SetInverted(TalonFXInvertType::OpposeMaster);

    lowerRight.SelectProfileSlot(0, 0);
    lowerRight.Config_kP(0, 0.08);
    lowerRight.Config_kI(0, 0);
    lowerRight.Config_kD(0, 0);

    /* Upper Motors */
    upperRight.ConfigAllSettings(baseConfig);
    upperRight.EnableVoltageCompensation(true);
    upperRight.SetNeutralMode(NeutralMode::Brake);

    upperLeft.ConfigAllSettings(baseConfig);
    upperLeft.EnableVoltageCompensation(true);
    upperLeft.SetNeutralMode(NeutralMode::Brake);

    upperRight.SetInverted(true);
    upperLeft.SetInverted(InvertType::FollowMaster);
    upperLeft.Follow(upperRight);
    upperLeft.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 255);
    upperLeft.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 255);


    upperRight.SelectProfileSlot(0, 0);
    upperRight.Config_kP(0, 0.012);
    upperRight.Config_kI(0, 0);
    upperRight.Config_kD(0, 0);
}

void DoubleArm::ConfigureSensors() {
    lowerRight.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    upperRight.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);

    std::this_thread::sleep_for(std::chrono::seconds(2));
    lowerRight.SetSelectedSensorPosition(ConvertAngleToLowerFalconPos(GetLowerAngle()));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    upperRight.SetSelectedSensorPosition(ConvertAngleToUpperFalconPos(GetUpperAngle()));
}

frc::Rotation2d DoubleArm::GetLowerAngle() {
    return units::degree_t((lowerEncoder.GetAbsolutePosition() - 0.3470692) * 360.0);
}

frc::Rotation2d DoubleArm::GetUpperAngle() {
    double rawVal = (-upperEncoder.GetAbsolutePosition() + 0.3033954) * 360.0;
    rawVal = frc::InputModulus(rawVal, -180.0, 180.0);
    frc::Rotation2d angleToLowerArm = frc::Rotation2d(units::degree_t(rawVal));

    // frc::SmartDashboard::PutNumber("DoubleArm/AngleUpperToLower", angleToLowerArm.Degrees().value());
    return angleToLowerArm + GetLowerAngle();
}

double DoubleArm::ConvertAngleToLowerFalconPos(frc::Rotation2d angle) {
    return angle.Degrees().value() / 360 * CODES_PER_LOWER_ROTATION;
}
double DoubleArm::ConvertAngleToUpperFalconPos(frc::Rotation2d angle) {
    frc::Rotation2d targetAngle = angle - GetLowerAngle();
    return targetAngle.Degrees().value() / 360 * CODES_PER_UPPER_ROTATION;
}