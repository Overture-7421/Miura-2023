// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include <Subsystems/SwerveChassis/SwerveChassis.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Drive
  : public frc2::CommandHelper<frc2::CommandBase, Drive> {
public:
  Drive(SwerveChassis* swerveChassis, frc::Joystick* controller);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  SwerveChassis* m_swerveChassis;

  units::meters_per_second_squared_t acceleration{ 15 };
  frc::SlewRateLimiter<units::meters_per_second> xLimiter{ acceleration };
  frc::SlewRateLimiter<units::meters_per_second> yLimiter{ acceleration };

  frc::Joystick* joystick;
};
