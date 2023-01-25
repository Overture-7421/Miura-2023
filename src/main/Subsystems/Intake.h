// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>

class Intake : public frc2::SubsystemBase {
 public:
  Intake();
 
 
 void setPiston	(	bool 	off	);
  
  void Periodic() override;

 private:
 
 WPI_TalonFX leftIntakeMotor {13};
 WPI_TalonFX rightIntakeMotor  {14};  
 frc::Solenoid::Solenoid intakeSolenoid {frc::PneumaticsModuleType::CTREPCM, 1, 2};
 
};
