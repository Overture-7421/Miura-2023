// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionManager.h"

VisionManager::VisionManager(SwerveChassis* chassis): chassis(chassis) {
    frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance();
    if (allianceColor == frc::DriverStation::Alliance::kBlue) {
        tagLayout.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
    } else {
        tagLayout.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);
    }

    poseEstimator = new photonlib::PhotonPoseEstimator{ tagLayout, photonlib::PoseStrategy::CLOSEST_TO_REFERENCE_POSE, std::move(cameraEstimator), cameraToRobot };

}

void VisionManager::Periodic() {
    /*Calculate pose using AprilTags*/
    std::optional<photonlib::EstimatedRobotPose>poseResult = poseEstimator->Update();
    if (poseResult) {
        chassis->addVisionMeasurement(poseResult.value().estimatedPose.ToPose2d(), poseResult.value().timestamp);
    }

}
