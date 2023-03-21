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

static frc2::CommandPtr BarrierDouble(SwerveChassis* m_swerveChassis, DoubleArm* m_doubleArm, Intake* m_intake) {
    pathplanner::PathPlannerTrajectory dropAndMove = pathplanner::PathPlanner::loadPath("Barrier_DropAndMove", { 2.5_mps, 2.5_mps_sq });
    pathplanner::PathPlannerTrajectory pickSecondPiece = pathplanner::PathPlanner::loadPath("Barrier_PickUpSecondPiece", { 3_mps, 3_mps_sq });
    pathplanner::PathPlannerTrajectory dropSecondPieceP1 = pathplanner::PathPlanner::loadPath("Barrier_DropSecondPieceP1", { 3_mps, 3_mps_sq });
    pathplanner::PathPlannerTrajectory dropSecondPieceP2 = pathplanner::PathPlanner::loadPath("Barrier_DropSecondPieceP2", { 3_mps, 3_mps_sq });


    // Wrist Down - False
    // Wrist Up - True
    // Cone Open - False
    // Cone Closed - True

    return frc2::cmd::Sequence(

        /* Upper cone dropped  */
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 1.81_m, 0.50_m, {180_deg} });}).ToPtr(),
        SetCone(m_intake, true),
        SetArmCoordinate(m_doubleArm, Positions::portal, Speeds::portal).ToPtr(), //Portal
        SetIntakeSpeed(m_intake, 2.8).ToPtr(),
        frc2::WaitCommand(0.5_s),
        SetIntakeSpeed(m_intake, 0.0).ToPtr(),

        /* Closed Pose & move  */
        frc2::cmd::Parallel(
            AutoTrajectories(m_swerveChassis, dropAndMove, { 0.3,0,0 }, { -0.03,0,0 }, { 1,0,0 }).AsProxy(),
            frc2::cmd::Sequence(
                SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr() //Closed
            )
        ),

        /* Open intake */
        SetCone(m_intake, false).ToPtr(),

        /* Follow trajectory to arrive to cube while Ground Pose */
        frc2::cmd::Parallel(
            SetArmCoordinate(m_doubleArm, Positions::groundAuto, Speeds::groundAuto).ToPtr(), //Ground
            SetIntakeSpeed(m_intake, -6.0).ToPtr()
        ),
        frc2::cmd::Sequence(
            AutoTrajectories(m_swerveChassis, pickSecondPiece, { 0.3,0,0 }, { -0.03,0,0 }, { 1,0,0 }).AsProxy(),
            frc2::WaitCommand(0.2_s)
        ),

        /* Close intake */
        SetIntakeSpeed(m_intake, 0.0).ToPtr(),

        /* Follow trajectory P1 while Closed Pose */
        frc2::cmd::Parallel(
            AutoTrajectories(m_swerveChassis, dropSecondPieceP1, { 0.3,0,0 }, { -0.03,0,0 }, { 1,0,0 }).AsProxy(),
            SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr() // Closed
        ),

        /* Follow trajectory P2 to arrive to grid */
        AutoTrajectories(m_swerveChassis, dropSecondPieceP2, { 0.3,0,0 }, { -0.03,0,0 }, { 1,0,0 }).AsProxy(),

        /* Upper cube dropped */
        SetWrist(m_intake, false).ToPtr(),
        SetArmCoordinate(m_doubleArm, Positions::middle, Speeds::middle).ToPtr(), //Middle
        SetIntakeSpeed(m_intake, 4.0),
        frc2::WaitCommand(0.5_s),
        SetIntakeSpeed(m_intake, 0.0).ToPtr(),

        /* Closed Pose */
        SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr() //Closed
    );
}