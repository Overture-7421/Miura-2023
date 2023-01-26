// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/InstantCommand.h>

#include "Subsystems/SwerveChassis/SwerveChassis.h"
#include "Subsystems/VisionManager/VisionManager.h"
#include "Commands/Teleop/Drive/Drive.h"

class RobotContainer {
public:
	RobotContainer();

	frc2::CommandPtr GetAutonomousCommand();

private:
	void ConfigureBindings();

	//Controllers
	frc::XboxController controller{ 0 };
	frc2::Trigger resetNavx{ [this] {return controller.GetYButton();} };

	// Subsystems
	SwerveChassis swerveChassis;
	// VisionManager visionManager{ &swerveChassis };

	//Auto

};
