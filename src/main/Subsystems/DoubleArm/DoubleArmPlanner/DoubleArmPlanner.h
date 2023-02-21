// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/geometry/Translation2d.h>
#include <units/length.h>
#include "../DoubleArmKinematics/DoubleArmState.h"
#include "../DoubleArmKinematics/DoubleArmKinematics.h"

typedef frc::TrapezoidProfile<units::meters> PlannerProfile;
class DoubleArmPlanner {
public:
    DoubleArmPlanner(PlannerProfile::Constraints constraints, DoubleArmKinematics kinematics);
    void SetTargetCoord(frc::Translation2d targetCoord, frc::Translation2d currentCoord);
    std::optional<DoubleArmState> CalculateCurrentTargetState();
    bool IsFinished();
private:
    units::second_t trajectoryStartTime;
    frc::Rotation2d trajectoryAngle;
    frc::Translation2d startingCoord;

    PlannerProfile::Constraints constraints;
    PlannerProfile targetPointProfile;
    DoubleArmKinematics kinematics;
};
