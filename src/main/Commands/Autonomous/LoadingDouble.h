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

#include <Subsystems/DoubleArm/ArmConstants.h>

using namespace ArmConstants;

static frc2::CommandPtr LoadingDouble(SwerveChassis* m_swerveChassis, DoubleArm* m_doubleArm, Intake* m_intake, pathplanner::SwerveAutoBuilder* autoBuilder) {
    std::vector<pathplanner::PathPlannerTrajectory> doubleLoadingTrajectory = pathplanner::PathPlanner::loadPathGroup("LoadingDouble", {
        {3_mps, 3_mps_sq },
        {3_mps, 3_mps_sq },
        {3_mps, 3_mps_sq }
        });


    return frc2::cmd::Sequence(
        /* Wrist Down, Upper Pose, Open Intake  */
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 1.81_m, 4.97_m, {180_deg} });}).ToPtr(),
        SetWrist(m_intake, false).ToPtr(),
        SetArmCoordinate(m_doubleArm, Positions::upper, Speeds::upper).ToPtr(), //Upper
        SetCone(m_intake, true).ToPtr(),

        /* Follow Trajectory 0 while Wrist Up and Closed Pose */
        frc2::cmd::Parallel(
            autoBuilder->followPath(doubleLoadingTrajectory[0]),
            SetWrist(m_intake, true).ToPtr(),
            SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr() //Closed
        ),

        /* Follow Trajectory 1 while Ground Pose  */
        frc2::cmd::Parallel(
            autoBuilder->followPath(doubleLoadingTrajectory[1]),
            SetArmCoordinate(m_doubleArm, Positions::ground, Speeds::ground).ToPtr() //Ground
        ),

        /* Close Intake */
        SetCone(m_intake, false).ToPtr(),

        /* Follow Trajectory 2 while Closed Pose */
        frc2::cmd::Parallel(
            autoBuilder->followPath(doubleLoadingTrajectory[2]),
            SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr() //Closed
        ),

        /* Wrist Down, Middle Pose and Open Intake */
        SetWrist(m_intake, false).ToPtr(),
        SetArmCoordinate(m_doubleArm, Positions::middle, Speeds::middle).ToPtr(), //Middle
        SetCone(m_intake, true).ToPtr(),

        /* Closed Pose */
        SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr(), //Closed

        /* Wrist Up and Intake Closed */
        frc2::cmd::Parallel(
            SetWrist(m_intake, true).ToPtr(),
            SetCone(m_intake, false).ToPtr()
        )
    );
}