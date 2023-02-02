// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Intake.h"

Intake::Intake() = default;


void Intake::setVoltage (double voltage) {
  leftIntakeMaster.SetVoltage(intakeVoltage);
} 

 void Intake::setPistons (bool state) {
    if (state) {
        intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
 } 


// This method will be called once per scheduler run
void Intake::Periodic() {}
