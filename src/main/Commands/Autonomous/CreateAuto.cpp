// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CreateAuto.h"

CreateAuto::CreateAuto(SwerveChassis* swerveChassis, DoubleArm* m_DoubleArm, Intake* m_Intake):
    m_SwerveChassis(swerveChassis), m_DoubleArm(m_DoubleArm), m_Intake(m_Intake) {

}

void CreateAuto::addCommandsToMap() {
    // Cone Piston
    frc2::InstantCommand coneOpen{ [intake = m_Intake]() { intake->setConeAuto(true);} };
    frc2::InstantCommand coneClosed{ [intake = m_Intake]() { intake->setConeAuto(false);} };

    // Wrist Piston
    frc2::InstantCommand wristUp{ [intake = m_Intake]() { intake->setWristAuto(true);} };
    frc2::InstantCommand wristDown{ [intake = m_Intake]() { intake->setWristAuto(false);} };

    frc2::InstantCommand closedPos{ [doubleArm = m_DoubleArm]() { doubleArm->SetTargetCoord({ 0.21_m, 0.05_m });} };
    frc2::InstantCommand groundPos{ [doubleArm = m_DoubleArm]() { doubleArm->SetTargetCoord({ 1_m, -.73_m });} };
    frc2::InstantCommand middlePos{ [doubleArm = m_DoubleArm]() { doubleArm->SetTargetCoord({ 0.76_m, 0.22_m });} };
    frc2::InstantCommand upperPos{ [doubleArm = m_DoubleArm]() { doubleArm->SetTargetCoord({ 1.2_m, 0.63_m });} };

    frc2::InstantCommand intakePiece{ [intake = m_Intake]() { intake->setVoltage(-6.0);} };
    frc2::InstantCommand stopIntake{ [intake = m_Intake]() { intake->setVoltage(0);} };

    eventMap.emplace("intakePiece", std::make_shared<frc2::InstantCommand>(intakePiece));
    eventMap.emplace("stopIntake", std::make_shared<frc2::InstantCommand>(stopIntake));
    eventMap.emplace("coneOpen", std::make_shared<frc2::InstantCommand>(coneOpen));
    eventMap.emplace("coneClosed", std::make_shared<frc2::InstantCommand>(coneClosed));
    eventMap.emplace("wristUp", std::make_shared<frc2::InstantCommand>(wristUp));
    eventMap.emplace("wristDown", std::make_shared<frc2::InstantCommand>(wristDown));
    eventMap.emplace("closedPos", std::make_shared<frc2::InstantCommand>(closedPos));
    eventMap.emplace("groundPos", std::make_shared<frc2::InstantCommand>(groundPos));
    eventMap.emplace("middlePos", std::make_shared<frc2::InstantCommand>(middlePos));
    eventMap.emplace("upperPos", std::make_shared<frc2::InstantCommand>(upperPos));
    eventMap.emplace("wait-0.5s", std::make_shared<frc2::WaitCommand>(0.5_s));
    eventMap.emplace("wait-1s", std::make_shared<frc2::WaitCommand>(1_s));
    eventMap.emplace("wait-1.5s", std::make_shared<frc2::WaitCommand>(1.5_s));
    eventMap.emplace("wait-2s", std::make_shared<frc2::WaitCommand>(2_s));
}

frc2::CommandPtr CreateAuto::GenerateAuto() {
    std::vector<pathplanner::PathPlannerTrajectory> path = pathplanner::PathPlanner::loadPathGroup(pathChooser.GetSelected(), { pathplanner::PathConstraints(3_mps, 2_mps_sq) });
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

    frc2::CommandPtr autoCommand = autoBuilder.fullAuto(path);

    return autoCommand;
}

return frc2::cmd::Print("No autonomous command configured");
}
