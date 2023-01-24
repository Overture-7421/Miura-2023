// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {

  swerveChassis.SetDefaultCommand(Drive(&swerveChassis, &controller));

  autoChooser.AddOption("Practice", PracticeAuto(&swerveChassis, paths->practicePaths, { 4_mps, 4_mps_sq }).ToPtr());

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return autoChooser.GetSelected();
}
