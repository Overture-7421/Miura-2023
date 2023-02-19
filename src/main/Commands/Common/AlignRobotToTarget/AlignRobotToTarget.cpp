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
    std::optional<photonlib::PhotonPipelineResult> cameraResult = visionManager->getCameraResult();
    if (cameraResult.has_value()) {
        photonlib::PhotonPipelineResult result = cameraResult.value();
        if (result.HasTargets()) {
            int tagID = result.GetBestTarget().GetFiducialId();
            frc::Pose2d targetpose = visionManager->getField().GetTagPose(tagID).value().ToPose2d();

            frc::Pose2d visionPose = targetpose.TransformBy({ positionMap[position].Translation(), positionMap[position].Rotation() });
            frc::Pose2d chassisPose = swerveChassis->getOdometry();

            // Create PathPoints
            std::vector<pathplanner::PathPoint> pathPoints = {
                { chassisPose.Translation(), chassisPose.Rotation(), chassisPose.Rotation() },
                { visionPose.Translation(), visionPose.Rotation(), visionPose.Rotation() }
            };

            // Create trajector with constraints
            trajectory = pathplanner::PathPlanner::generatePath(constraints, pathPoints);
        } else if (!result.HasTargets()) {
            End(true);
        }
    } else if (!cameraResult.has_value()) {
        End(true);
    }

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
        false
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
