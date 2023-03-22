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

static frc2::CommandPtr LoadingDouble(SwerveChassis* m_swerveChassis, DoubleArm* m_doubleArm, Intake* m_intake) {
    pathplanner::PathPlannerTrajectory dropAndMove = pathplanner::PathPlanner::loadPath("Loading_P1", { 3_mps, 2.5_mps_sq });
    pathplanner::PathPlannerTrajectory pickSecondPiece = pathplanner::PathPlanner::loadPath("Loading_P2", { 3_mps, 2.5_mps_sq });
    pathplanner::PathPlannerTrajectory dropSecond = pathplanner::PathPlanner::loadPath("Loading_P3", { 3_mps, 2.5_mps_sq });

    // Wrist Down - False
    // Wrist Up - True
    // Cone Open - True
    // Cone Closed - False

    return frc2::cmd::Sequence(
        /* Upper cone dropped  */
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 1.81_m, 4.97_m, {180_deg} });}).ToPtr(),
        SetArmCoordinate(m_doubleArm, Positions::middle, Speeds::middle).ToPtr(), //Portal
        SetWrist(m_intake, false).ToPtr(),
        SetCone(m_intake, true).ToPtr(),
        frc2::WaitCommand(0.3_s),
        SetWrist(m_intake, false).ToPtr(),

        /* Closed Pose & move */
        frc2::cmd::Parallel(
            AutoTrajectories(m_swerveChassis, dropAndMove, { 0.3,0,0 }, { -0.03,0,0 }, { 1,0,0 }).AsProxy(),
            SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr() //Closed
        ),

        /* Follow trajectory to pick cube while Ground Pose  */
        frc2::cmd::Parallel(
            SetCone(m_intake, false).ToPtr(),
            SetArmCoordinate(m_doubleArm, Positions::groundAuto, Speeds::groundAuto).ToPtr() //Ground
        ),
        frc2::cmd::Parallel(
            SetIntakeSpeed(m_intake, -6.0).ToPtr(),
            frc2::cmd::Sequence(
                AutoTrajectories(m_swerveChassis, pickSecondPiece, { 0.3,0,0 }, { 0.12,0,0 }, { 1,0,0 }).AsProxy(),
                frc2::WaitCommand(0.4_s)
            )
        ),

        /* Follow trajectory to arrive to grid while Closed Pose */
        frc2::cmd::Parallel(
            AutoTrajectories(m_swerveChassis, dropSecond, { 0.29,0,0 }, { -0.05,0,0 }, { 1.37,0,0 }).AsProxy(), //1.25
            frc2::cmd::Sequence(
                SetArmCoordinate(m_doubleArm, Positions::closedauto, Speeds::closedauto).ToPtr(), // Closed
                SetArmCoordinate(m_doubleArm, Positions::armInvertedAuto, Speeds::armInvertedAuto).ToPtr() //ArmInvertedAuto
            )
        ),

        /* Upper cube dropped */
        frc2::WaitCommand(0.3_s),
        SetIntakeSpeed(m_intake, 8.1),
        frc2::WaitCommand(0.5_s),
        SetIntakeSpeed(m_intake, 0.0).ToPtr(),

        /* Closed Pose */
        SetArmCoordinate(m_doubleArm, Positions::closedauto, Speeds::closedauto).ToPtr() //Closed
    );
}