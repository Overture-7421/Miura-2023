// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

std::vector<pathplanner::PathPlannerTrajectory> RobotContainer::outBarrierTrajectory = pathplanner::PathPlanner::loadPathGroup("OutBarrier", { pathplanner::PathConstraints(3_mps, 2_mps_sq) });
std::vector<pathplanner::PathPlannerTrajectory> RobotContainer::outLoadingTrajectory = pathplanner::PathPlanner::loadPathGroup("OutLoading", { pathplanner::PathConstraints(3_mps, 2_mps_sq) });
std::vector<pathplanner::PathPlannerTrajectory> RobotContainer::outCenterTrajectory = pathplanner::PathPlanner::loadPathGroup("OutCenter", { pathplanner::PathConstraints(3_mps, 2_mps_sq) });
std::vector<pathplanner::PathPlannerTrajectory> RobotContainer::barrier1PieceTrajectory = pathplanner::PathPlanner::loadPathGroup("Barrier_1_Piece", { pathplanner::PathConstraints(3_mps, 2_mps_sq) });
std::vector<pathplanner::PathPlannerTrajectory> RobotContainer::loading1PieceTrajectory = pathplanner::PathPlanner::loadPathGroup("Loading_1_Piece", { pathplanner::PathConstraints(3_mps, 2_mps_sq) });
std::vector<pathplanner::PathPlannerTrajectory> RobotContainer::center1PieceTrajectory = pathplanner::PathPlanner::loadPathGroup("Center_1_Piece", { pathplanner::PathConstraints(3_mps, 2_mps_sq) });

RobotContainer::RobotContainer():
	autoBuilder(
		[this]() { return swerveChassis.getOdometry(); },
		[this](auto initPose) { swerveChassis.resetOdometry(initPose); },
		swerveChassis.getKinematics(),
		pathplanner::PIDConstants(5.0, 0.0, 0.0),
		pathplanner::PIDConstants(0.5, 0.0, 0.0),
		[this](auto speeds) { swerveChassis.setModuleStates(speeds); },
		eventMap,
		{ &swerveChassis },
		true
	),
	outBarrier(autoBuilder.fullAuto(outBarrierTrajectory)),
	outCenter(autoBuilder.fullAuto(outCenterTrajectory)),
	outLoading(autoBuilder.fullAuto(outLoadingTrajectory)),
	barrier1Piece(autoBuilder.fullAuto(barrier1PieceTrajectory)),
	loading1Piece(autoBuilder.fullAuto(loading1PieceTrajectory)),
	center1Piece(autoBuilder.fullAuto(center1PieceTrajectory)) {
	//Set default commands
	swerveChassis.SetDefaultCommand(Drive(&swerveChassis, &controller));
	intake.SetDefaultCommand(IntakeControl(&intake, &mechanisms));

	//Set choosers for auto
	pathChooser.AddOption("OutBarrier", outBarrier.get());
	pathChooser.AddOption("OutLoading", outLoading.get());
	pathChooser.AddOption("OutCenter", outCenter.get());
	pathChooser.AddOption("Barrier1Piece", barrier1Piece.get());
	pathChooser.AddOption("Loading1Piece", loading1Piece.get());
	pathChooser.AddOption("Center1Piece", center1Piece.get());
	pathChooser.SetDefaultOption("None", nullptr);
	frc::SmartDashboard::PutData("Auto Chooser", &pathChooser);

	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
	// Chassis Controller Buttons
	resetNavx.OnTrue(frc2::InstantCommand{ [this]() {this->swerveChassis.resetNavx();} }.ToPtr());
	alignOneLeft.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "1-Left").ToPtr());
	alignOneCenter.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "1-Center").ToPtr());
	alignOneRight.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "1-Right").ToPtr());
	alignTwoLeft.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "2-Left").ToPtr());
	alignTwoCenter.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "2-Center").ToPtr());
	alignTwoRight.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "2-Right").ToPtr());
	alignThreeLeft.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "3-Left").ToPtr());
	alignThreeCenter.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "3-Center").ToPtr());
	alignThreeRight.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "3-Right").ToPtr());
	alignLoading.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "Loading").ToPtr());
	autoBalance.WhileTrue(AutoBalance(&swerveChassis).ToPtr());

	// Mechanisms Controller Buttons
	conePiston.OnTrue(frc2::InstantCommand{ [this]() { this->intake.setConeControl();} }.ToPtr());
	wristPiston.OnTrue(frc2::InstantCommand{ [this]() { this->intake.setWristControl();} }.ToPtr());

	lowerPosition.OnTrue(frc2::SequentialCommandGroup{
		frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({ 0.21_m, 0.05_m });} },
			frc2::WaitCommand{(1_s)},
			frc2::InstantCommand{ [this]() { this->intake.setWristAuto(false);} },
	}.ToPtr()); // Closed

	groundPickUp.OnTrue(frc2::SequentialCommandGroup{
		frc2::InstantCommand{ [this]() { this->intake.setWristAuto(true);} },
			frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({ 1_m, -.73_m });} },
	}.ToPtr()); // Ground

	middlePosition.OnTrue(frc2::SequentialCommandGroup{
		frc2::InstantCommand{ [this]() { this->intake.setWristAuto(false);} },
			frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({ 0.76_m, 0.22_m });} }
	}.ToPtr()); // Middle

	upperPosition.OnTrue(frc2::SequentialCommandGroup{
		frc2::InstantCommand{ [this]() { this->intake.setWristAuto(true);} },
			frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({ 1.2_m, 0.63_m });} }
	}.ToPtr()); // upper


	portalPosition.OnTrue(frc2::SequentialCommandGroup{
		frc2::InstantCommand{ [this]() { this->intake.setWristAuto(false);} },
			frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({ 0.82_m, 0.23_m });} }
	}.ToPtr()); // Portal
}

void RobotContainer::setVisionManager() {
	frc::DriverStation::Alliance color = frc::DriverStation::GetAlliance();
	if (color != frc::DriverStation::Alliance::kInvalid && frc::DriverStation::IsDSAttached()) {
		visionManager.setAllianceColor();
	}
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	return pathChooser.GetSelected();
}