// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.


#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>

#include "Subsystems/DoubleArm/DoubleArm.h"
#include "Subsystems/Intake/Intake.h"
#include "Subsystems/SwerveChassis/SwerveChassis.h"
#include "Commands/Common/SetWrist/SetWrist.h"
#include "Commands/Common/SetCone/SetCone.h"
#include "Commands/Common/SetintakeSpeed/SetIntakeSpeed.h"
#include "Commands/Common/SetArmCoordinate/SetArmCoordinate.h"

#include <Subsystems/DoubleArm/ArmConstants.h>

using namespace ArmConstants;

static frc2::CommandPtr LoadingMiddle(SwerveChassis* m_swerveChassis, DoubleArm* m_doubleArm, Intake* m_intake, pathplanner::SwerveAutoBuilder* autoBuilder) {
    std::vector<pathplanner::PathPlannerTrajectory> outLoadingTrajectory = pathplanner::PathPlanner::loadPathGroup("OutLoading", { pathplanner::PathConstraints(2_mps, 1.5_mps_sq) });


    return frc2::cmd::Sequence(
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 1.81_m, 4.97_m, {180_deg} });}).ToPtr(),
        SetWrist(m_intake, false).ToPtr(),
        SetArmCoordinate(m_doubleArm, Positions::middle, Speeds::middle).ToPtr(), //Middle
        SetCone(m_intake, true).ToPtr(),
        frc2::WaitCommand{ 0.5_s }.ToPtr(),
        SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr(), //Closed
        autoBuilder->followPath(outLoadingTrajectory[0])
    );
}

