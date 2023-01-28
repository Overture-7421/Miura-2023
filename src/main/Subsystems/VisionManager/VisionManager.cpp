// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionManager.h"

VisionManager::VisionManager(SwerveChassis* chassis): chassis(chassis) {

};

void VisionManager::Periodic() {
    cameraResult = camera.GetLatestResult();

    // if (cameraResult.HasTargets()) {
    //     photonlib::PhotonTrackedTarget target = cameraResult.GetBestTarget();
    //     int targetID = target.GetFiducialId();
    //     frc::Pose3d targetPose = tagLayout.GetTagPose(targetID).value();

        // frc::Pose2d visionPose{};
        // frc::SmartDashboard::PutNumber("Vision/Vision-X", 10000);
        //     if (poseEstimator.Update().has_value()) {
        //         frc::SmartDashboard::PutNumber("Vision/Vision-Y", 99999);
        //         photonlib::EstimatedRobotPose poseResult = poseEstimator.Update().value();
        //         units::millisecond_t timestamp = (frc::Timer::GetFPGATimestamp() - cameraResult.GetLatency());
        //         visionPose = poseResult.estimatedPose.ToPose2d();
        //         chassis->addVisionMeasurement(visionPose, timestamp);
        //     }

        //     frc::SmartDashboard::PutNumber("Vision/Vision-X", visionPose.X().value());
        //     frc::SmartDashboard::PutNumber("Vision/Vision-Y", visionPose.Y().value());
        //     frc::SmartDashboard::PutNumber("Vision/Vision-Degrees", visionPose.Rotation().Degrees().value());

        // frc::SmartDashboard::PutNumber("Vision/Target-X", targetPose.X().value());
        // frc::SmartDashboard::PutNumber("Vision/Target-Y", targetPose.Y().value());
        // frc::SmartDashboard::PutNumber("Vision/Target-Degrees", targetPose.Rotation().ToRotation2d().Degrees().value());
    // }
}