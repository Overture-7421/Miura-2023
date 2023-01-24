// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Subsystems/SwerveChassis/SwerveChassis.h"
#include "Subsystems/VisionManager/VisionManager.h"
#include "Commands/Teleop/Drive/Drive.h"
#include "Commands/Paths/Paths.h"
#include "Commands/Autonomous/PracticeAuto.h"

class RobotContainer {
public:
	RobotContainer();

	frc2::Command* GetAutonomousCommand();

private:
	void ConfigureBindings();

	//Controllers
	frc::Joystick controller{ 0 };

	// Subsystems
	SwerveChassis swerveChassis;
	// VisionManager visionManager{ &swerveChassis };

	//Auto
	frc::SendableChooser<frc2::Command*> autoChooser;
	Paths* paths;

	PracticeAuto practiceAuto{ &swerveChassis, paths->practicePaths, { 4_mps, 4_mps_sq } };

};
