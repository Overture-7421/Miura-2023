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
    std::vector<pathplanner::PathPlannerTrajectory> doubleLoadingTrajectory = pathplanner::PathPlanner::loadPathGroup("LoadingDouble", { pathplanner::PathConstraints(3_mps, 3_mps_sq) });


    return frc2::cmd::Sequence(
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 1.81_m, 4.97_m, {180_deg} });}).ToPtr(),
        SetWrist(m_intake, false).ToPtr(),
        SetArmCoordinate(m_doubleArm, Positions::middle, Speeds::middle).ToPtr(), //Middle
        SetCone(m_intake, true).ToPtr(),
        frc2::cmd::Parallel(
            SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr(), //Closed
            frc2::cmd::Sequence(
                autoBuilder->followPath(doubleLoadingTrajectory[0]),
                frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->setSpeed({ 0_mps, 0_mps, 0_deg });}, { m_swerveChassis }).ToPtr()
            )
        ),
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->setSpeed({ 0_mps, 0_mps, 0_deg });}).ToPtr(),
        SetWrist(m_intake, true).ToPtr(),
        SetCone(m_intake, true).ToPtr(),
        frc2::cmd::Parallel(
            SetArmCoordinate(m_doubleArm, Positions::ground, Speeds::ground).ToPtr(), //Ground
            frc2::cmd::Sequence(
                autoBuilder->followPath(doubleLoadingTrajectory[1]),
                frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->setSpeed({ 0_mps, 0_mps, 0_deg });}, { m_swerveChassis }).ToPtr()
            )
        ),
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->setSpeed({ 0_mps, 0_mps, 0_deg });}).ToPtr(),
        SetCone(m_intake, false).ToPtr(),
        frc2::cmd::Parallel(
            frc2::cmd::Sequence(
                SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr(), //Closed
                SetWrist(m_intake, false).ToPtr()
            ),
            frc2::cmd::Sequence(
                autoBuilder->followPath(doubleLoadingTrajectory[2]),
                frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->setSpeed({ 0_mps, 0_mps, 0_deg });}, { m_swerveChassis }).ToPtr()
            )
        ),
        SetCone(m_intake, true).ToPtr()
    );
}