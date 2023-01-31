// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionManager.h"

VisionManager::VisionManager(SwerveChassis* chassis): chassis(chassis) {

};

void VisionManager::Periodic() {
    /*Calculate pose using AprilTags*/
    std::optional<photonlib::EstimatedRobotPose>poseResult = poseEstimator.Update();
    if (poseResult) {
        chassis->addVisionMeasurement(poseResult.value().estimatedPose.ToPose2d(), poseResult.value().timestamp);
    }

}