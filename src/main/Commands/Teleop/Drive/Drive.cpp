// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drive.h"

Drive::Drive(SwerveChassis* swerveChassis, frc::Joystick* controller): m_swerveChassis(swerveChassis), joystick(controller) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_swerveChassis);
}

// Called when the command is initially scheduled.
void Drive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Drive::Execute() {
  frc::ChassisSpeeds chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    units::meters_per_second_t{ -joystick->GetRawAxis(1) * 5 },
    units::meters_per_second_t{ -joystick->GetRawAxis(0) * 5 },
    units::radians_per_second_t(-joystick->GetRawAxis(4) * 9),
    m_swerveChassis->getOdometry().Rotation());


  m_swerveChassis->setSpeed(chassisSpeeds.vx.value(), chassisSpeeds.vy.value(), chassisSpeeds.omega.value());
}

// Called once the command ends or is interrupted.
void Drive::End(bool interrupted) {}

// Returns true when the command should end.
bool Drive::IsFinished() {
  return false;
}
