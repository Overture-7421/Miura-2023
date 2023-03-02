// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CreateAuto.h"

CreateAuto::CreateAuto(SwerveChassis* swerveChassis, DoubleArm* m_DoubleArm, Intake* m_Intake):
    m_SwerveChassis(swerveChassis), m_DoubleArm(m_DoubleArm), m_Intake(m_Intake) {
    //Set choosers for auto
    pathChooser.SetDefaultOption("LoadingSideAuto", "LoadingSideAuto");
    frc::SmartDashboard::PutData("Auto Chooser", &pathChooser);
}

void CreateAuto::addCommandsToMap() {
    // Cone Piston
    frc2::InstantCommand coneOpen{ [this]() { this->m_Intake->setConeAuto(true);} };
    frc2::InstantCommand coneClosed{ [this]() { this->m_Intake->setConeAuto(false);} };

    // Wrist Piston
    frc2::InstantCommand wristUp{ [this]() { this->m_Intake->setWristAuto(true);} };
    frc2::InstantCommand wristDown{ [this]() { this->m_Intake->setWristAuto(false);} };

    frc2::InstantCommand closedPos{ [this]() { this->m_DoubleArm->SetTargetCoord({ 0.19_m, 0.03_m });} };
    frc2::InstantCommand groundPos{ [this]() { this->m_DoubleArm->SetTargetCoord({  1_m, -.75_m });} };
    frc2::InstantCommand bottomPos{ [this]() {this->m_DoubleArm->SetTargetCoord({  0.76_m, 0.2_m });} };
    frc2::InstantCommand upperPos{ [this]() {this->m_DoubleArm->SetTargetCoord({  1.15_m, 0.62_m });} };

    eventMap.emplace("coneOpen", std::make_shared<frc2::InstantCommand>(coneOpen));
    eventMap.emplace("coneClosed", std::make_shared<frc2::InstantCommand>(coneClosed));
    eventMap.emplace("wristUp", std::make_shared<frc2::InstantCommand>(wristUp));
    eventMap.emplace("wristDown", std::make_shared<frc2::InstantCommand>(wristDown));
    eventMap.emplace("closedPos", std::make_shared<frc2::InstantCommand>(closedPos));
    eventMap.emplace("groundPos", std::make_shared<frc2::InstantCommand>(groundPos));
    eventMap.emplace("bottomPos", std::make_shared<frc2::InstantCommand>(bottomPos));
    eventMap.emplace("upperPos", std::make_shared<frc2::InstantCommand>(upperPos));
}

frc2::CommandPtr CreateAuto::GenerateAuto() {
    addCommandsToMap();
    std::vector<pathplanner::PathPlannerTrajectory> examplePath = pathplanner::PathPlanner::loadPathGroup(pathChooser.GetSelected(), { pathplanner::PathConstraints(4_mps, 4_mps_sq),
    pathplanner::PathConstraints(4_mps, 4_mps_sq),
    pathplanner::PathConstraints(.5_mps, 2_mps_sq), });


    pathplanner::SwerveAutoBuilder autoBuilder(
        [this]() { return m_SwerveChassis->getOdometry(); }, // Function to supply current robot pose
        [this](auto initPose) { m_SwerveChassis->resetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
        m_SwerveChassis->getKinematics(),
        pathplanner::PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        pathplanner::PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        [this](auto speeds) { m_SwerveChassis->setModuleStates(speeds); }, // Output function that accepts field relative ChassisSpeeds
        eventMap, // Our event map
        { m_SwerveChassis }, // Drive requirements, usually just a single drive subsystem
        true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    );

    frc2::CommandPtr autoCommand = autoBuilder.fullAuto(examplePath);

    return autoCommand;
}
