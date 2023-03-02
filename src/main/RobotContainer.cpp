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
            frc2::InstantCommand{ [this]() { this->intake.setWristAuto(true);} },
            frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({ 0.19_m, 0.03_m });} }
        }.ToPtr()); // Closed

    groundPickUp.OnTrue(frc2::SequentialCommandGroup{
        frc2::InstantCommand{ [this]() { this->intake.setWristAuto(true);} },
        frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({  1_m, -.75_m });} }
        }.ToPtr()); // Ground

    middlePosition.OnTrue(frc2::SequentialCommandGroup{
    frc2::InstantCommand{ [this]() { this->intake.setWristAuto(false);} },
    frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({  0.76_m, 0.2_m });} }
        }.ToPtr()); // Middle

    upperPosition.OnTrue(frc2::SequentialCommandGroup{
        frc2::InstantCommand{ [this]() { this->intake.setWristAuto(true);} },
        frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({  1.15_m, 0.62_m });} }
        }.ToPtr()); // upper


    portalPosition.OnTrue(frc2::SequentialCommandGroup{
        frc2::InstantCommand{ [this]() { this->intake.setWristAuto(false);} },
        frc2::InstantCommand{ [this]() {this->doubleArm.SetTargetCoord({  0.82_m, 0.23_m  });} }
        }.ToPtr()); // Portal
}

void RobotContainer::setVisionManager() {
    frc::DriverStation::Alliance color = frc::DriverStation::GetAlliance();
    if (color != frc::DriverStation::Alliance::kInvalid && frc::DriverStation::IsDSAttached()) {
        visionManager.setAllianceColor();
    }

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    // Set alliance color for pose estimation and correct on the fly path generation
    return createAuto.GenerateAuto();
}