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

static frc2::CommandPtr DropConeAndMove(SwerveChassis* m_swerveChassis, DoubleArm* m_doubleArm, Intake* m_intake) {
    pathplanner::PathPlannerTrajectory moveAndClose = pathplanner::PathPlanner::loadPath("DropCone_P1", { 3_mps, 2.5_mps_sq });
    pathplanner::PathPlannerTrajectory outCommunity = pathplanner::PathPlanner::loadPath("DropCone_P2", { 2_mps, 2_mps_sq });


    // Wrist Down - False
    // Wrist Up - True
    // Cone Open - True
    // Cone Closed - False

    return frc2::cmd::Sequence(
        /* Upper cone dropped  */
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 1.81_m, 2.23_m, {180_deg} });}).ToPtr(),
        SetWrist(m_intake, false).ToPtr(),
        // SetArmCoordinate(m_doubleArm, Positions::upper, Speeds::upper).ToPtr(), //Upper
        SetCone(m_intake, false).ToPtr(),
        frc2::WaitCommand{ 0.5_s }.ToPtr(),

        AutoTrajectories(m_swerveChassis, moveAndClose, { 0.3,0,0 }, { 0,0,0 }, { 1.27,0,0 }).AsProxy(),

        // SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr(), //Closed

        AutoTrajectories(m_swerveChassis, outCommunity, { 0.3,0,0 }, { 0,0,0 }, { 1.27,0,0 }).AsProxy()

    );
}