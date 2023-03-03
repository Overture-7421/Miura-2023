// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/XboxController.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>

#include "Subsystems/SwerveChassis/SwerveChassis.h"
#include "Subsystems/VisionManager/VisionManager.h"
#include "Subsystems/DoubleArm/DoubleArm.h"
#include "Subsystems/Intake/Intake.h"

#include "Commands/Teleop/Drive/Drive.h"
#include "Commands/Common/AlignRobotToTarget/AlignRobotToTarget.h"
#include "Commands/Common/IntakeControl/IntakeControl.h"
#include "Commands/Common/AutoBalance/AutoBalance.h"

class RobotContainer {
public:
	RobotContainer();
	frc2::Command* GetAutonomousCommand();
	void setVisionManager();

private:
	void ConfigureBindings();

	// Chassis driver controller and buttons
	frc::XboxController controller{ 0 };
	frc2::Trigger resetNavx{ [this] {return controller.GetBackButton();} };
	frc2::Trigger alignOneLeft{ [this] {return controller.GetLeftBumper() && controller.GetXButton();} };
	frc2::Trigger alignOneCenter{ [this] {return controller.GetLeftBumper() && controller.GetAButton();} };
	frc2::Trigger alignOneRight{ [this] {return controller.GetLeftBumper() && controller.GetBButton();} };
	frc2::Trigger alignTwoLeft{ [this] {return (controller.GetLeftTriggerAxis() || controller.GetRightTriggerAxis()) && controller.GetXButton();} };
	frc2::Trigger alignTwoCenter{ [this] {return (controller.GetLeftTriggerAxis() || controller.GetRightTriggerAxis()) && controller.GetAButton();} };
	frc2::Trigger alignTwoRight{ [this] {return (controller.GetLeftTriggerAxis() || controller.GetRightTriggerAxis()) && controller.GetBButton();} };
	frc2::Trigger alignThreeLeft{ [this] {return controller.GetRightBumper() && controller.GetXButton();} };
	frc2::Trigger alignThreeCenter{ [this] {return controller.GetRightBumper() && controller.GetAButton();} };
	frc2::Trigger alignThreeRight{ [this] {return controller.GetRightBumper() && controller.GetBButton();} };
	frc2::Trigger alignLoading{ [this] {return controller.GetYButton();} };
	frc2::Trigger autoBalance{ [this] {return controller.GetStartButton();} };

	// Mechanism Controller
	frc::XboxController mechanisms{ 1 };

	//Pneumatics
	frc2::Trigger conePiston{ [this] {return mechanisms.GetRightBumper();} };
	frc2::Trigger wristPiston{ [this] {return mechanisms.GetLeftBumper();} };

	//Positions
	frc2::Trigger upperPosition{ [this] {return mechanisms.GetYButton();} }; //Upper
	frc2::Trigger middlePosition{ [this] {return mechanisms.GetXButton();} }; //Middle
	frc2::Trigger groundPickUp{ [this] {return mechanisms.GetAButton();} }; //Ground
	frc2::Trigger portalPosition{ [this] {return mechanisms.GetBButton();} }; //Portal
	frc2::Trigger lowerPosition{ [this] {return mechanisms.GetPOV(0.75);} }; //Closed

	// Subsystems
	SwerveChassis swerveChassis;
	VisionManager visionManager{ &swerveChassis };
	Intake intake;
	DoubleArm doubleArm;

	// Commands


	// Auto
	frc::SendableChooser<frc2::Command*> pathChooser;
	pathplanner::SwerveAutoBuilder autoBuilder;
	std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap{
		{"DropUpperCone", std::make_shared<frc2::SequentialCommandGroup>(
			frc2::InstantCommand{ [this]() { this->intake.setWristAuto(true);} },
			frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({ 1.2_m, 0.63_m });} },
			frc2::WaitCommand{ 2_s },
			frc2::InstantCommand{ [this]() {this->intake.setWristAuto(false);} },
			frc2::InstantCommand{ [this]() {this->intake.setConeAuto(false);} }
		)},
		{ "DropMiddleCone", std::make_shared<frc2::SequentialCommandGroup>(
			frc2::InstantCommand{ [this]() { this->intake.setWristAuto(true);} },
			frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({ 0.76_m, 0.22_m });} },
			frc2::WaitCommand{ 2_s },
			frc2::InstantCommand{ [this]() {this->intake.setWristAuto(false);} },
			frc2::InstantCommand{ [this]() {this->intake.setConeAuto(false);} }
		) },
		{ "CloseArm", std::make_shared<frc2::SequentialCommandGroup>(
			frc2::InstantCommand{ [this]() {this->intake.setVoltage(0);} },
			frc2::InstantCommand{ [this]() { this->intake.setWristAuto(true);} },
			frc2::InstantCommand{ [this]() { this->intake.setConeAuto(true);} },
			frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({ 0.21_m, 0.05_m });} },
			frc2::WaitCommand{ 2_s }
		) },
		{ "GroundPickUp", std::make_shared<frc2::SequentialCommandGroup>(
			frc2::InstantCommand{ [this]() { this->intake.setWristAuto(true);} },
			frc2::InstantCommand{ [this]() { this->intake.setConeAuto(true);} },
			frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({ 1_m, -.73_m });} },
			frc2::WaitCommand{ 2_s },
			frc2::InstantCommand{ [this]() {this->intake.setVoltage(-4);} }
		) }
	};

	static std::vector<pathplanner::PathPlannerTrajectory> outBarrierTrajectory;
	static std::vector<pathplanner::PathPlannerTrajectory> outCenterTrajectory;
	static std::vector<pathplanner::PathPlannerTrajectory> outLoadingTrajectory;
	static std::vector<pathplanner::PathPlannerTrajectory> barrier1PieceTrajectory;
	static std::vector<pathplanner::PathPlannerTrajectory> loading1PieceTrajectory;
	static std::vector<pathplanner::PathPlannerTrajectory> center1PieceTrajectory;

	frc2::CommandPtr outBarrier;
	frc2::CommandPtr outCenter;
	frc2::CommandPtr outLoading;
	frc2::CommandPtr barrier1Piece;
	frc2::CommandPtr loading1Piece;
	frc2::CommandPtr center1Piece;
};
