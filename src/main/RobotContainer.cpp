// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
    //Set default commands
    swerveChassis.SetDefaultCommand(Drive(&swerveChassis, &controller));
    visionManager.SetDefaultCommand(UpdateVisionOdometry(&visionManager));
    intake.SetDefaultCommand(IntakeControl(&intake, &mechanisms));

    //Set choosers for auto
    pathChooser.SetDefaultOption("OutOfCommunity&Balance", "OutOfCommunity&Balance");
    frc::SmartDashboard::PutData("Auto Chooser", &pathChooser);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    // Chassis Controller Buttons
    resetNavx.OnTrue(frc2::InstantCommand{ [this]() {this->swerveChassis.resetNavx();} }.ToPtr());
    alignCenter.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "Center").ToPtr());
    alignRight.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "Right").ToPtr());
    alignLeft.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "Left").ToPtr());

    // Mechanisms Controller Buttons
    conePiston.OnTrue(frc2::InstantCommand{ [this]() { this->intake.setConeControl();} }.ToPtr());
    wristPiston.OnTrue(frc2::InstantCommand{ [this]() { this->intake.setWristControl();} }.ToPtr());
    lowerPosition.OnTrue(frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({ 0.25_m, 0.13_m });} }.ToPtr()); // Closed
    groundPickUp.OnTrue(frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({ 1.45_m, -0.42_m });} }.ToPtr()); // Ground
    middlePosition.OnTrue(frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({ 0.83_m, 0.25_m });} }.ToPtr()); // Middle
    upperPosition.OnTrue(frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({ 1.22_m, 0.78_m });} }.ToPtr()); // Upper

}

void RobotContainer::setVisionManager() {
    frc::DriverStation::Alliance color = frc::DriverStation::GetAlliance();
    if (color != frc::DriverStation::Alliance::kInvalid && frc::DriverStation::IsDSAttached()) {
        visionManager.setAllianceColor();
    }

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    // Set alliance color for pose estimation and correct on the fly path generation
    return CreateAuto(pathChooser.GetSelected());
}

frc2::CommandPtr RobotContainer::CreateAuto(std::string pathName) {
    std::vector<pathplanner::PathPlannerTrajectory> examplePath = pathplanner::PathPlanner::loadPathGroup(pathName, { pathplanner::PathConstraints(4_mps, 4_mps_sq),
    pathplanner::PathConstraints(4_mps, 4_mps_sq),
    pathplanner::PathConstraints(.5_mps, 2_mps_sq), });

    eventMap.emplace("Stage1", std::make_shared<frc2::PrintCommand>("Salió en la terminal??"));


    pathplanner::SwerveAutoBuilder autoBuilder(
        [this]() { return swerveChassis.getOdometry(); }, // Function to supply current robot pose
        [this](auto initPose) { swerveChassis.resetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
        swerveChassis.getKinematics(),
        pathplanner::PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        pathplanner::PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        [this](auto speeds) { swerveChassis.setModuleStates(speeds); }, // Output function that accepts field relative ChassisSpeeds
        eventMap, // Our event map
        { &swerveChassis }, // Drive requirements, usually just a single drive subsystem
        true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    );

    frc2::CommandPtr autoCommand = autoBuilder.fullAuto(examplePath);

    return autoCommand;
}
