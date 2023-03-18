#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
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

#include "Commands/Autonomous/AutoTrajectories/AutoTrajectories.h"

#include <Subsystems/DoubleArm/ArmConstants.h>

using namespace ArmConstants;

static frc2::CommandPtr PIDConfig(SwerveChassis* m_swerveChassis, DoubleArm* m_doubleArm, Intake* m_intake, pathplanner::SwerveAutoBuilder* autoBuilder) {
    pathplanner::PathPlannerTrajectory practicePID1Rot = pathplanner::PathPlanner::loadPath("Practice PID1 Rotation", { 3_mps, 3_mps_sq });
    pathplanner::PathPlannerTrajectory practicePID2Rot = pathplanner::PathPlanner::loadPath("Practice PID2 Rotation", { 3_mps, 3_mps_sq });

    return frc2::cmd::Sequence(
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 2.0_m, 4.8_m, {0_deg} });}).ToPtr(),
        AutoTrajectories(m_swerveChassis, &practicePID1Rot).ToPtr()
    );
}