#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
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

static frc2::CommandPtr CenterBalance(SwerveChassis* m_swerveChassis, DoubleArm* m_doubleArm, Intake* m_intake) {
    pathplanner::PathPlannerTrajectory balance = pathplanner::PathPlanner::loadPath("CenterBalance", { 3_mps, 2.5_mps_sq });

    // Wrist Down - False
    // Wrist Up - True
    // Cone Open - True
    // Cone Closed - False

    return frc2::cmd::Sequence(
        /* Upper cone dropped  */
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 1.81_m, 2.75_m, {0_deg} });}).ToPtr(),
        SetCone(m_intake, false),
        SetArmCoordinate(m_doubleArm, Positions::armInvertedAuto, Speeds::armInvertedAuto).ToPtr(), //ArmInvertedAuto

        /* Upper cube dropped */
        frc2::WaitCommand(0.3_s),
        SetIntakeSpeed(m_intake, ArmConstants::AutoPieces::AutoTopCube).ToPtr(),
        frc2::WaitCommand(0.5_s),
        SetIntakeSpeed(m_intake, 0.0).ToPtr(),

        /* Closed Pose */
        SetArmCoordinate(m_doubleArm, Positions::closedauto, Speeds::closedauto).ToPtr(), // Closed

        /* Balance Trajectory */
        AutoTrajectories(m_swerveChassis, balance, { 0.3,0,0 }, { 0,0,0 }, { 1.25,0,0 }).AsProxy()
    );
}