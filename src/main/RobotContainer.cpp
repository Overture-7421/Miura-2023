// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ███    ███ ██ ██    ██ ██████   █████      ██████   ██████  ██████   ██████  ████████      ██████  ██████  ██████  ███████
// ████  ████ ██ ██    ██ ██   ██ ██   ██     ██   ██ ██    ██ ██   ██ ██    ██    ██        ██      ██    ██ ██   ██ ██     
// ██ ████ ██ ██ ██    ██ ██████  ███████     ██████  ██    ██ ██████  ██    ██    ██        ██      ██    ██ ██   ██ █████
// ██  ██  ██ ██ ██    ██ ██   ██ ██   ██     ██   ██ ██    ██ ██   ██ ██    ██    ██        ██      ██    ██ ██   ██ ██      
// ██      ██ ██  ██████  ██   ██ ██   ██     ██   ██  ██████  ██████   ██████     ██         ██████  ██████  ██████  ███████

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {

    //Set default commands
    intake.SetDefaultCommand(IntakeControl(&intake, &mechanisms));

    //Set choosers for auto
    pathChooser.AddOption("Loading Cubes", loadingDouble.get());
    pathChooser.AddOption("Loading Cubes & Balance", loadingBalance.get());
    pathChooser.AddOption("Barrier Cubes", barrierDouble.get());
    pathChooser.AddOption("Barrier Cubes & Balance", barrierBalance.get());
    pathChooser.AddOption("Center Balance", centerBalance.get());
    pathChooser.AddOption("Drop Cone", dropCone.get());
    pathChooser.AddOption("Drop Cone and Move", dropConeAndMove.get());


    pathChooser.SetDefaultOption("None", nullptr);
    frc::SmartDashboard::PutData("Auto Chooser", &pathChooser);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    // Chassis Controller Buttons
    resetNavx.OnTrue(frc2::InstantCommand{ [this]() {this->swerveChassis.resetNavx();} }.ToPtr());
    autoBalance.WhileTrue(AutoBalance(&swerveChassis).ToPtr());

    // Mechanisms Controller Buttons
    conePiston.OnTrue(frc2::InstantCommand{ [this]() { this->intake.setConeControl();} }.ToPtr());
    wristPiston.OnTrue(frc2::InstantCommand{ [this]() { this->intake.setWristControl();} }.ToPtr());

    lowerPosition.OnTrue(frc2::SequentialCommandGroup{
        SetWrist(&intake, true),
        SetArmCoordinate(&doubleArm, Positions::StartingPosition)
        }.ToPtr()); // Closed

    groundPickUp.OnTrue(frc2::SequentialCommandGroup{
        SetWrist(&intake, true),
        SetArmCoordinate(&doubleArm, Positions::StartingPosition),
        }.ToPtr()); // Ground

    middlePosition.OnTrue(frc2::SequentialCommandGroup{
        SetWrist(&intake, true),
        SetArmCoordinate(&doubleArm, Positions::MiddlePosition),
        }.ToPtr()); // Middle

    upperPosition.OnTrue(frc2::SequentialCommandGroup{
        SetWrist(&intake, false),
        SetArmCoordinate(&doubleArm, Positions::StartingPosition)
        }.ToPtr()); // Upper

    portalPosition.OnTrue(frc2::SequentialCommandGroup{
        SetWrist(&intake, false),
        SetArmCoordinate(&doubleArm, Positions::StartingPosition)
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