// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AutoTrajectories.h"

AutoTrajectories::AutoTrajectories(SwerveChassis* swerveChassis, pathplanner::PathPlannerTrajectory trajectory, frc2::PIDController xController, frc2::PIDController yController, frc2::PIDController rController):
    swerveChassis(swerveChassis), m_trajectory(trajectory), m_xController(xController), m_yController(yController), m_rController(rController) {
    AddRequirements({ swerveChassis });
    m_trajectory = trajectory;
}

// Called when the command is initially scheduled.
void AutoTrajectories::Initialize() {
    alignCommand = new pathplanner::PPSwerveControllerCommand(
        m_trajectory,
        [this]() { return swerveChassis->getOdometry(); },
        swerveChassis->getKinematics(),
        m_xController,
        m_yController,
        m_rController,
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
