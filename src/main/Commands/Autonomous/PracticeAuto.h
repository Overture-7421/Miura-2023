// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Subsystems/SwerveChassis/SwerveChassis.h"

#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc2/command/InstantCommand.h>
#include <numbers>

class PracticeAuto
  : public frc2::CommandHelper<frc2::SequentialCommandGroup,
  PracticeAuto> {
public:
  PracticeAuto(SwerveChassis* swerveChassis, std::vector<frc::Pose2d> positions, frc::TrajectoryConfig trajectoryConfig);
private:
  SwerveChassis* swerveChassis;

};
