#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/PrintCommand.h>
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
#include "Commands/Common/AutoBalanceRotate/AutoBalanceRotate.h"
#include "Commands/Common/AutoBalance/AutoBalance.h"

#include <Subsystems/DoubleArm/ArmConstants.h>

using namespace ArmConstants;

static frc2::CommandPtr CenterBalance(SwerveChassis* m_swerveChassis, DoubleArm* m_doubleArm, Intake* m_intake) {
    pathplanner::PathPlannerTrajectory leavegrid = pathplanner::PathPlanner::loadPath("Center_Balance_P1", { 3_mps, 2.5_mps_sq });
    pathplanner::PathPlannerTrajectory balance = pathplanner::PathPlanner::loadPath("Center_Balance_P2", { 4_mps, 4_mps_sq });

    // Wrist Down - False
    // Wrist Up - True
    // Cone Open - True
    // Cone Closed - False

    return frc2::cmd::Sequence(

        /* Odometry */
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 1.81_m, 2.23_m, {0_deg} });}).ToPtr(),
        SetCone(m_intake, true),

        /* Leave Grid */
        AutoTrajectories(m_swerveChassis, leavegrid, { 0.3,0,0 }, { 0,0,0 }, { 1.25,0,0 }),
        AutoBalanceRotate(m_swerveChassis, 55).ToPtr().WithTimeout(1.5_s),

        /* Rotate and Balance Trajectory */
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 2.20_m, 2.23_m, {55_deg} });}).ToPtr(),
        AutoTrajectories(m_swerveChassis, balance, { 0.4,0,0 }, { 0,0,0 }, { 1.25,0,0 }),

        frc2::WaitCommand(0.5_s),

        /*************** DANGER AUTOBALANCE **************/
        AutoBalance(m_swerveChassis).ToPtr()
        /*************** DANGER AUTOBALANCE **************/
    );
}