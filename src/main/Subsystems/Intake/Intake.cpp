// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake() {
    intakeMotor.SetNeutralMode(NeutralMode::Brake);
    intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 20);
    intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 255);
    intakeMotor.ConfigOpenloopRamp(0, 1);
    intakeMotor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 25, 0, 1));
}

void Intake::setVoltage(double voltage) {
    intakeMotor.SetVoltage(units::volt_t(voltage));
}

void Intake::setConeControl() {
    conePiston.Toggle();
}

void Intake::setConeAuto(bool state) {
    if (state) {
        conePiston.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        conePiston.Set(frc::DoubleSolenoid::Value::kReverse);
    }
}

void Intake::setWristControl() {
    wristPiston.Toggle();
}

void Intake::setWristAuto(bool state) {
    if (state) {
        wristPiston.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        wristPiston.Set(frc::DoubleSolenoid::Value::kReverse);
    }
}

double Intake::getUltrasonic() {
    double rawValue = ultrasonic.GetValue();
    double scaleFactor = 5 / frc::RobotController::GetVoltage5V();
    double distanceCentimeters = rawValue * scaleFactor * 0.125; //forzado?
    return distanceCentimeters;
}

// This method will be called once per scheduler run
void Intake::Periodic() {
    frc::SmartDashboard::PutNumber("Sensor", getUltrasonic());
}
