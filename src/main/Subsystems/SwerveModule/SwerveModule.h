// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>


class SwerveModule: public frc2::SubsystemBase {
public:
  SwerveModule(int rotatorID, int wheelID, int canCoderID, double offSet, std::string name = "");
  double getSpeed();
  double getDistance();
  double getMeters(double codes);
  void SetRotatorVoltage(double rotatorVoltage);
  void SetWheelVoltage(double wheelVoltage);
  double getAngle();
  double getRotatorPID(double setPoint);
  double getWheelPID(double setPoint);
  frc::SwerveModuleState getState();
  void setState(frc::SwerveModuleState state);
  frc::SwerveModulePosition getPosition();
  void setRotatorPIDValues(double kP, double kI, double kD, double f);
  void setWheelPIDValues(double kP, double kI, double kD, double f);
  void setUseRawVoltageSpeed(bool set);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  //Declaration of motors
  WPI_TalonFX rotator;
  WPI_TalonFX wheel;
  double wheelVoltage = 0;

  //Declaration of CanCoder
  CANCoder canCoder;

  //PID
  frc2::PIDController rotatorPID{ 0.125, 0.5, 0 };
  frc2::PIDController wheelPID{ 0.1, 0, 0 };
  double rotatorF = 0;
  double wheelF = 0;

  //State
  frc::SwerveModuleState moduleState;

  double offSet;
  std::string name;
  bool useRawVoltageSpeed = false;
};


