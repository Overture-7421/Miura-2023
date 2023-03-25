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
#include "Commands/Common/AutoBalanceRotate/AutoBalanceRotate.h"
#include "Commands/Common/AutoBalance/AutoBalance.h"


#include <Subsystems/DoubleArm/ArmConstants.h>

using namespace ArmConstants;

static frc2::CommandPtr LoadingBalance(SwerveChassis* m_swerveChassis, DoubleArm* m_doubleArm, Intake* m_intake) {
    pathplanner::PathPlannerTrajectory pickSecondPiece = pathplanner::PathPlanner::loadPath("Loading_P1", { 3.5_mps, 3.5_mps_sq });
    pathplanner::PathPlannerTrajectory dropSecond = pathplanner::PathPlanner::loadPath("Loading_P2", { 4_mps, 3.5_mps_sq });
    pathplanner::PathPlannerTrajectory moveBalance = pathplanner::PathPlanner::loadPath("Loading_P3", { 4_mps, 3.5_mps_sq });
    pathplanner::PathPlannerTrajectory balance = pathplanner::PathPlanner::loadPath("Loading_P4", { 4_mps, 4_mps_sq });

    // Wrist Down - False
    // Wrist Up - True
    // Cone Open - True
    // Cone Closed - False

    return frc2::cmd::Sequence(
        /* Odometry and Arm Position  */
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 1.81_m, 4.43_m, {0_deg} });}).ToPtr(),
        SetArmCoordinate(m_doubleArm, Positions::armInvertedAuto, Speeds::armInvertedAuto).ToPtr(), //ArmInvertedAuto

        /* Upper cube dropped */
        frc2::WaitCommand(0.3_s),
        SetIntakeSpeed(m_intake, ArmConstants::AutoPieces::AutoTopCube),
        frc2::WaitCommand(0.5_s),
        SetIntakeSpeed(m_intake, 0.0).ToPtr(),

        /* Follow trajectory to pick cube while Ground Pose  */

        frc2::cmd::Parallel(
            frc2::cmd::Sequence(
                // SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr(), //Closed
                SetArmCoordinate(m_doubleArm, Positions::groundAuto, Speeds::closedauto).ToPtr() //Ground
            ),
            SetIntakeSpeed(m_intake, -6.0).ToPtr(),
            frc2::cmd::Sequence(
                frc2::WaitCommand(1.3_s),
                AutoTrajectories(m_swerveChassis, pickSecondPiece, { 0.4,0,0 }, { 0.01,0,0 }, { 1,0,0 }).AsProxy(), //-0.1
                frc2::WaitCommand(0.4_s)
            )
        ),

        /* Follow trajectory to arrive to grid while Closed Pose */
        frc2::cmd::Parallel(
            AutoTrajectories(m_swerveChassis, dropSecond, { 0.5,0,0 }, { 0.02,0,0 }, { 1.27,0,0 }).AsProxy(),
            frc2::cmd::Sequence(
                SetArmCoordinate(m_doubleArm, Positions::closedauto, Speeds::closedauto).ToPtr(), // Closed
                SetArmCoordinate(m_doubleArm, Positions::armInvertedAuto, Speeds::armInvertedAuto).ToPtr() //ArmInvertedAuto
            )
        ),

        /* Upper cube dropped */
        frc2::WaitCommand(0.3_s),
        SetIntakeSpeed(m_intake, ArmConstants::AutoPieces::AutoMiddleCube),
        frc2::WaitCommand(0.5_s),
        SetIntakeSpeed(m_intake, 0.0).ToPtr(),

        /* Closed Pose */
        frc2::cmd::Parallel(
            AutoTrajectories(m_swerveChassis, moveBalance, { 0.3,0,0 }, { -0.3,0,0 }, { 1.25,0,0 }).AsProxy(),
            SetArmCoordinate(m_doubleArm, Positions::closedauto, Speeds::closedauto).ToPtr() //Closed
        ),

        /*************** ROTATION **************/
        AutoBalanceRotate(m_swerveChassis, 55).ToPtr().WithTimeout(1.5_s),



        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 2.29_m, 3.15_m, {55_deg} });}).ToPtr(),

        AutoTrajectories(m_swerveChassis, balance, { 0.4,0,0 }, { 0,0,0 }, { 1.25,0,0 }).AsProxy(),

        frc2::WaitCommand(0.5_s),

        /*************** DANGER AUTOBALANCE **************/
        AutoBalance(m_swerveChassis).ToPtr()
        /*************** DANGER AUTOBALANCE **************/

    );
}