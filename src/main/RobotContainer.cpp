// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {

    //Set default commands
    intake.SetDefaultCommand(IntakeControl(&intake, &mechanisms));

    //Set choosers for auto
    pathChooser.AddOption("Loading Middle", loadingMiddle.get());
    pathChooser.AddOption("Barrier Middle", barrierMiddle.get());
    // pathChooser.AddOption("Loading Double", loadingDouble.get());


    pathChooser.SetDefaultOption("None", nullptr);
    frc::SmartDashboard::PutData("Auto Chooser", &pathChooser);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    // Chassis Controller Buttons
    resetNavx.OnTrue(frc2::InstantCommand{ [this]() {this->swerveChassis.resetNavx();} }.ToPtr());
    // alignOneLeft.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "1-Left").ToPtr());
    // alignOneCenter.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "1-Center").ToPtr());
    // alignOneRight.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "1-Right").ToPtr());
    // alignTwoLeft.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "2-Left").ToPtr());
    // alignTwoCenter.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "2-Center").ToPtr());
    // alignTwoRight.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "2-Right").ToPtr());
    // alignThreeLeft.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "3-Left").ToPtr());
    // alignThreeCenter.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "3-Center").ToPtr());
    // alignThreeRight.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "3-Right").ToPtr());
    // alignLoading.WhileTrue(AlignRobotToTarget(&swerveChassis, &visionManager, "Loading").ToPtr());
    autoBalance.WhileTrue(AutoBalance(&swerveChassis).ToPtr());

    // Mechanisms Controller Buttons
    conePiston.OnTrue(frc2::InstantCommand{ [this]() { this->intake.setConeControl();} }.ToPtr());
    wristPiston.OnTrue(frc2::InstantCommand{ [this]() { this->intake.setWristControl();} }.ToPtr());

    lowerPosition.OnTrue(frc2::SequentialCommandGroup{
        SetArmCoordinate(&doubleArm, Positions::closed, Speeds::closed),
        frc2::WaitCommand{(1_s)},
        SetWrist(&intake, false)
        }.ToPtr()); // Closed

    groundPickUp.OnTrue(frc2::SequentialCommandGroup{
        SetWrist(&intake, true),
        SetArmCoordinate(&doubleArm, Positions::ground, Speeds::ground),
        }.ToPtr()); // Ground

    middlePosition.OnTrue(frc2::SequentialCommandGroup{
        SetWrist(&intake, false),
        SetArmCoordinate(&doubleArm, Positions::middle, Speeds::middle),
        }.ToPtr()); // Middle

    upperPosition.OnTrue(frc2::SequentialCommandGroup{
        SetWrist(&intake, false),
        SetArmCoordinate(&doubleArm, Positions::upper, Speeds::upper),
        SetWrist(&intake, true)
        }.ToPtr()); // upper

    portalPosition.OnTrue(frc2::SequentialCommandGroup{
        SetWrist(&intake, false),
        SetArmCoordinate(&doubleArm, Positions::portal, Speeds::portal),
        }.ToPtr()); // Portal
}

// void RobotContainer::setVisionManager() {
//     frc::DriverStation::Alliance color = frc::DriverStation::GetAlliance();
//     if (color != frc::DriverStation::Alliance::kInvalid && frc::DriverStation::IsDSAttached()) {
//         visionManager.setAllianceColor();
//     }
// }

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return pathChooser.GetSelected();
}

void RobotContainer::setDriveCommand() {
    swerveChassis.SetDefaultCommand(Drive(&swerveChassis, &controller));
}