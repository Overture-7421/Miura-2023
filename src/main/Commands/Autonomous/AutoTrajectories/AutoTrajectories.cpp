// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AutoTrajectories.h"

AutoTrajectories::AutoTrajectories(SwerveChassis* swerveChassis, pathplanner::PathPlannerTrajectory trajectory): swerveChassis(swerveChassis), m_trajectory(trajectory) {
    AddRequirements({ swerveChassis });
    m_trajectory = trajectory;
}

// Called when the command is initially scheduled.
void AutoTrajectories::Initialize() {
    alignCommand = new pathplanner::PPSwerveControllerCommand(
        m_trajectory,
        [this]() { return swerveChassis->getOdometry(); },
        swerveChassis->getKinematics(),
        { 0.3,0,0 },
        { -0.013,0,0 },
        { 0.5,0,0 },
        [this](auto speeds) { swerveChassis->setModuleStates(speeds); },
        { swerveChassis },
        true
    );

    alignCommand->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutoTrajectories::Execute() {
    alignCommand->Execute();
}

// Called once the command ends or is interrupted.
void AutoTrajectories::End(bool interrupted) {
    alignCommand->End(interrupted);
}

// Returns true when the command should end.
bool AutoTrajectories::IsFinished() {
    return alignCommand->IsFinished();
}
