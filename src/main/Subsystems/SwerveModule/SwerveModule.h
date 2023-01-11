// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModuleState.h>


class SwerveModule : public frc2::SubsystemBase {
 public:
  SwerveModule(int rotatorID, int wheelID, int canCoderID, double offSet);
  double getSpeed();
  double getMeters(double codes);
  void SetRotatorVoltage(double rotatorVoltage);
  void SetWheelVoltage(double wheelVoltage);
  double getAngle();
  double getPID(double setPoint);
  void setAngle(double angle);
  frc::SwerveModuleState getState();
  void setPIDvalues(double kP, double kI, double kD, double f);
  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
//Declaration of motors
  WPI_TalonFX rotator;
  WPI_TalonFX wheel;

//Declaration of CanCoder
  CANCoder canCoder;
  frc2::PIDController rotatorPID{0.125, 0.5, 0};
  double angle = 0;
  double f = 0;
  double offSet;
};


