// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PathPlannerCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
PathPlannerCommand::PathPlannerCommand(SwerveChassis* swerveChassis): swerveChassis(swerveChassis) {
	// Add your commands here, e.g.
	// AddCommands(FooCommand{}, BarCommand{});

	pathplanner::PathPlannerTrajectory examplePath = pathplanner::PathPlanner::loadPath("Example Path", pathplanner::PathConstraints(4_mps, 3_mps_sq));

	std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

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

	frc2::CommandPtr command = autoBuilder.fullAuto(examplePath);
	// AddCommands(command, stopCommand);


}
