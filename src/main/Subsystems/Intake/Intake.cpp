// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake() {
  intakeMotor.SetNeutralMode(NeutralMode::Brake);
  intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 20);
  intakeMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 255);
  intakeMotor.ConfigOpenloopRamp(0,1);
  intakeMotor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 25, 0, 1));
}

void Intake::setVoltage (double voltage) {
  intakeMotor.SetVoltage(units::volt_t(voltage));
} 

 void Intake::setConeControl() {
  coneSolenoid.Toggle();
 }

 void Intake::setConeAuto(bool state) {
    if (state) {
        coneSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        coneSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
 } 

 void Intake::setWristControl() {
  wristSolenoid.Toggle();
 }
 
 void Intake::setWristAuto(bool state) {
    if (state) {
        wristSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        wristSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
 } 

// This method will be called once per scheduler run
void Intake::Periodic() {}
