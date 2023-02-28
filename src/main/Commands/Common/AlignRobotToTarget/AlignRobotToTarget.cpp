// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AlignRobotToTarget.h"

AlignRobotToTarget::AlignRobotToTarget(SwerveChassis* swerveChassis, VisionManager* visionManager, std::string position)
    : swerveChassis(swerveChassis), visionManager(visionManager), position(position) {
    AddRequirements({ swerveChassis, visionManager });
}

/* Method for calculating the desiredPosition for path generation */
void AlignRobotToTarget::calculateAlignPose() {

    frc::Pose2d targetPose = positionMap[position];
    frc::Pose2d chassisPose = swerveChassis->getOdometry();

    // Create PathPoints
    std::vector<pathplanner::PathPoint> pathPoints = {
        { chassisPose.Translation(), {0_deg}, chassisPose.Rotation() },
        { targetPose.Translation(), {0_deg}, targetPose.Rotation() }
    };

    // Create trajector with constraints
    trajectory = pathplanner::PathPlanner::generatePath(constraints, pathPoints);
}

// Called when the command is initially scheduled.
void AlignRobotToTarget::Initialize() {
    calculateAlignPose();

    alignCommand = new pathplanner::PPSwerveControllerCommand(
        trajectory,
        [this]() { return swerveChassis->getOdometry(); },
        swerveChassis->getKinematics(),
        { 0,0,0 },
        { 0,0,0 },
        { 0,0,0 },
        [this](auto speeds) { swerveChassis->setModuleStates(speeds); },
        { swerveChassis },
        true
    );

    alignCommand->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AlignRobotToTarget::Execute() {
    alignCommand->Execute();
}

// Called once the command ends or is interrupted.
void AlignRobotToTarget::End(bool interrupted) {
    alignCommand->End(interrupted);
}

// Returns true when the command should end.
bool AlignRobotToTarget::IsFinished() {
    return alignCommand->IsFinished();
}
