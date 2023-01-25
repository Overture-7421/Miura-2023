// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {

	swerveChassis.SetDefaultCommand(Drive(&swerveChassis, &controller));

	autoChooser.SetDefaultOption("Practice", &practiceAuto);
	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
	resetNavx.OnTrue(frc2::InstantCommand{ [this]() {this->swerveChassis.resetNavx();} }.ToPtr());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return autoChooser.GetSelected();
}
