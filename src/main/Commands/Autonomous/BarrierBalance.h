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

static frc2::CommandPtr BarrierBalance(SwerveChassis* m_swerveChassis, DoubleArm* m_doubleArm, Intake* m_intake) {
    pathplanner::PathPlannerTrajectory dropAndMove = pathplanner::PathPlanner::loadPath("Barrier_DropAndMove", { 2_mps, 2_mps_sq });
    pathplanner::PathPlannerTrajectory pickSecondPiece = pathplanner::PathPlanner::loadPath("Barrier_PickUpSecondPiece", { 3_mps, 3_mps_sq });
    pathplanner::PathPlannerTrajectory alignForBalance = pathplanner::PathPlanner::loadPath("Barrier_AlignForBalance", { 2.5_mps, 2.5_mps_sq });
    pathplanner::PathPlannerTrajectory balance = pathplanner::PathPlanner::loadPath("Barrier_Balance", { 2_mps, 2_mps_sq });

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

        //Closed Pose & move
        frc2::cmd::Parallel(
            AutoTrajectories(m_swerveChassis, dropAndMove).AsProxy(),
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
            AutoTrajectories(m_swerveChassis, pickSecondPiece).AsProxy(),
            frc2::WaitCommand(0.2_s)
        ),


        /* Close intake */
        SetIntakeSpeed(m_intake, 0.0).ToPtr(),


        /* Align for balance while Closed Pose */
        frc2::cmd::Parallel(
            AutoTrajectories(m_swerveChassis, alignForBalance).AsProxy(),
            SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr() // Closed
        ),

        /* Balance */
        AutoTrajectories(m_swerveChassis, balance).AsProxy()
    );
}