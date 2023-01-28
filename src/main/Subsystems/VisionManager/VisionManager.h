// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include "Subsystems/SwerveChassis/SwerveChassis.h"

class VisionManager: public frc2::SubsystemBase {
public:
  VisionManager(SwerveChassis* chassis);

  void Periodic() override;

private:
  photonlib::PhotonCamera cameraEstimator{ "IMX219" };
  photonlib::PhotonCamera camera{ "IMX219" };
  photonlib::PhotonPipelineResult cameraResult;
  frc::Transform3d cameraToRobot{ {0_m, 0_m, 0.50_m}, frc::Rotation3d{} };
  frc::AprilTagFieldLayout tagLayout{ frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp) };
  photonlib::PhotonPoseEstimator poseEstimator{ tagLayout, photonlib::PoseStrategy::CLOSEST_TO_REFERENCE_POSE, std::move(cameraEstimator), cameraToRobot };

  SwerveChassis* chassis;

};
