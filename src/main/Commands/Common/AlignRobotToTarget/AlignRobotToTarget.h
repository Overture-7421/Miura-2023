// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/WaitCommand.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPlanner.h>

#include "Subsystems/SwerveChassis/SwerveChassis.h"
#include "Subsystems/VisionManager/VisionManager.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AlignRobotToTarget
    : public frc2::CommandHelper<frc2::CommandBase, AlignRobotToTarget> {
public:
    AlignRobotToTarget(SwerveChassis* swerveChassis, VisionManager* visionManager, std::string position);
    void calculateAlignPose();

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    /* Subsystems */
    SwerveChassis* swerveChassis;
    VisionManager* visionManager;

    /* AlignRobot variables using pathplanner */
    std::string position;
    pathplanner::PPSwerveControllerCommand* alignCommand;
    pathplanner::PathPlannerTrajectory trajectory;
    pathplanner::PathConstraints constraints = { 2_mps, 2_mps_sq };
    units::meter_t xPosition{ 2.2_m };
    std::unordered_map<std::string, frc::Pose2d> positionMap{
      {"1-Left", {xPosition, 5_m, {180_deg}}},
      {"1-Center", {xPosition, 4.42_m, {180_deg}}},
      {"1-Right", {xPosition, 3.86_m, {180_deg}}},
      {"2-Left", {xPosition, 3.30_m, {180_deg}}},
      {"2-Center", {xPosition, 2.75_m, {180_deg}}},
      {"2-Right", {xPosition, 2.19_m, {180_deg}}},
      {"3-Left", {xPosition, 1.63_m, {180_deg}}},
      {"3-Center", {xPosition, 1.06_m, {180_deg}}},
      {"3-Right", {xPosition, 0.53_m, {180_deg}}},
      {"Loading",{14.5_m, 6.7_m, {0_deg}}}
    };
};
