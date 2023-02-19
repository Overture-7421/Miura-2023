// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionManager.h"

VisionManager::VisionManager(SwerveChassis* swerveChassis): swerveChassis(swerveChassis) {}

void VisionManager::setAllianceColor() {
    // Set alliance color for the poseEstimator
    frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance();
    if (allianceColor == frc::DriverStation::Alliance::kBlue) {
        tagLayout.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
    } else {
        tagLayout.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);
    }

    // Set pose estimator
    poseEstimator = new photonlib::PhotonPoseEstimator{
        tagLayout,
        photonlib::PoseStrategy::MULTI_TAG_PNP,
        std::move(cameraEstimator),
        cameraToRobot
    };

    poseEstimator->SetMultiTagFallbackStrategy(photonlib::PoseStrategy::LOWEST_AMBIGUITY);
}

//Update odometry with vision
void VisionManager::updateOdometry() {
    std::optional<photonlib::EstimatedRobotPose> poseResult = update(swerveChassis->getOdometry());

    if (poseResult.has_value()) {
        photonlib::EstimatedRobotPose pose = poseResult.value();
        swerveChassis->addVisionMeasurement(pose.estimatedPose.ToPose2d(), pose.timestamp);
    }
}

//Get EstimatedRobotPose from PhotonVision
std::optional<photonlib::EstimatedRobotPose> VisionManager::update(frc::Pose2d estimatedPose) {
    poseEstimator->SetReferencePose(frc::Pose3d(estimatedPose));
    return poseEstimator->Update();
}

//GetPhotonPipeLineResult from PhotonVision
std::optional<photonlib::PhotonPipelineResult> VisionManager::getCameraResult() {
    return camera.GetLatestResult();
}

//Get AprilTagFieldLayout from driver station
frc::AprilTagFieldLayout VisionManager::getField() {
    return tagLayout;
}

void VisionManager::Periodic() {}
