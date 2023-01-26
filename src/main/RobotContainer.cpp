// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {

	swerveChassis.SetDefaultCommand(Drive(&swerveChassis, &controller));

	pathChooser.SetDefaultOption("PracticePath", "PracticePath");

	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
	resetNavx.OnTrue(frc2::InstantCommand{ [this]() {this->swerveChassis.resetNavx();} }.ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	return CreateAuto(pathChooser.GetSelected());
}

frc2::CommandPtr RobotContainer::CreateAuto(std::string pathName) {
	pathplanner::PathPlannerTrajectory examplePath = pathplanner::PathPlanner::loadPath(pathName, pathplanner::PathConstraints(4_mps, 4_mps_sq));

	pathplanner::SwerveAutoBuilder autoBuilder(
		[this]() { return this->swerveChassis->getOdometry(); }, // Function to supply current robot pose
		[this](auto initPose) { this->swerveChassis->resetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
		swerveChassis->getKinematics(),
		pathplanner::PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
		pathplanner::PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
		[this](auto speeds) { this->swerveChassis->setModuleStates(speeds); }, // Output function that accepts field relative ChassisSpeeds
		eventMap, // Our event map
		{ swerveChassis }, // Drive requirements, usually just a single drive subsystem
		true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
	);

	frc2::CommandPtr autoCommand = autoBuilder.fullAuto(examplePath);

	return autoCommand;
}
