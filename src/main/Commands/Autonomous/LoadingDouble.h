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

#include <Subsystems/DoubleArm/ArmConstants.h>

using namespace ArmConstants;

static frc2::CommandPtr LoadingDouble(SwerveChassis* m_swerveChassis, DoubleArm* m_doubleArm, Intake* m_intake) {
    pathplanner::PathPlannerTrajectory loadAndMoveP1 = pathplanner::PathPlanner::loadPath("DropAndMoveP1", { 2.5_mps, 2.5_mps_sq });
    pathplanner::PathPlannerTrajectory loadAndMoveP2 = pathplanner::PathPlanner::loadPath("DropAndMoveP2", { 2.5_mps, 2.5_mps_sq });
    pathplanner::PathPlannerTrajectory pickSecondPiece = pathplanner::PathPlanner::loadPath("PickUpSecondPiece", { 3_mps, 3_mps_sq });
    pathplanner::PathPlannerTrajectory dropSecond = pathplanner::PathPlanner::loadPath("DropSecondPiece", { 3_mps, 3_mps_sq });

    // Wrist Down - False
    // Wrist Up - True
    // Cone Open - True
    // Cone Closed - False

    return frc2::cmd::Sequence(
        /* Upper Cone Left  */
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 1.81_m, 4.97_m, {180_deg} });}).ToPtr(),
        SetArmCoordinate(m_doubleArm, Positions::portal, Speeds::portal).ToPtr(), //Portal
        SetIntakeSpeed(m_intake, 3.0).ToPtr(),
        frc2::WaitCommand(0.3_s),
        SetIntakeSpeed(m_intake, 0.0).ToPtr(),

        //Close Arm
        //autoBuilder->followPath(loadAndMoveP1).AsProxy(),
        SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr(), //Colsed


        /* Trayectory before ground */
        //autoBuilder->followPath(loadAndMoveP2).AsProxy(),


        /* Follow Trajectory to pick while Ground Pose  */
        frc2::cmd::Parallel(
            //autoBuilder->followPath(pickSecondPiece).AsProxy(),
            SetArmCoordinate(m_doubleArm, Positions::ground, Speeds::ground).ToPtr() //Ground

        ),

        /* Close Intake */
        SetCone(m_intake, true).ToPtr(),

        /* Follow Trajectory to arrive to grid while Closed Pose */
        frc2::cmd::Parallel(
            SetCone(m_intake, false).ToPtr(),
            //autoBuilder->followPath(dropSecond).AsProxy(),
            SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr() //Closed
        ),

        /* Middle Cone Left */
        SetWrist(m_intake, false).ToPtr(),
        SetIntakeSpeed(m_intake, 3.0).ToPtr(),
        frc2::WaitCommand(0.3_s),
        SetIntakeSpeed(m_intake, 0.0).ToPtr(),
        SetWrist(m_intake, true).ToPtr()
    );
}