#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
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

#include "Commands/Common/SetArmCoordinate/SetArmCoordinate.h"

static frc2::CommandPtr LoadingDouble(SwerveChassis* m_swerveChassis, DoubleArm* m_doubleArm, Intake* m_intake) {
    std::vector<pathplanner::PathPlannerTrajectory> doubleLoadingTrajectory = pathplanner::PathPlanner::loadPathGroup("LoadingDouble", { pathplanner::PathConstraints(2_mps, 2_mps_sq) });
    pathplanner::SwerveAutoBuilder autoBuilder(
        [m_swerveChassis = m_swerveChassis]() { return m_swerveChassis->getOdometry(); },
        [m_swerveChassis = m_swerveChassis](auto initPose) { m_swerveChassis->resetOdometry(initPose); },
        m_swerveChassis->getKinematics(),
        pathplanner::PIDConstants(0.02, 0.0, 0.0),
        pathplanner::PIDConstants(0.01, 0.0, 0.0),
        [m_swerveChassis = m_swerveChassis](auto speeds) { m_swerveChassis->setModuleStates(speeds); },
        {},
        { m_swerveChassis },
        true
    );

    return frc2::cmd::Sequence(
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 1.8_m, 0.5_m, {180_deg} });}).ToPtr(),
        SetWrist(m_intake, false).ToPtr(),
        SetArmCoordinate(m_doubleArm, { 0.76_m, 0.22_m }).ToPtr(), //Middle
        SetCone(m_intake, true).ToPtr(),
        frc2::WaitCommand{ 0.5_s }.ToPtr(),
        SetArmCoordinate(m_doubleArm, { 0.21_m, 0.05_m }).ToPtr(), //Closed
        autoBuilder.followPath(doubleLoadingTrajectory[0]),
        SetWrist(m_intake, true).ToPtr(),
        SetCone(m_intake, true).ToPtr(),
        SetArmCoordinate(m_doubleArm, { 1_m, -.73_m }).ToPtr(), //Ground
        autoBuilder.followPath(doubleLoadingTrajectory[1]),
        SetCone(m_intake, false).ToPtr(),
        frc2::WaitCommand{ 0.5_s }.ToPtr(),
        SetArmCoordinate(m_doubleArm, { 0.21_m, 0.05_m }).ToPtr(), //Closed
        SetWrist(m_intake, false).ToPtr(),
        autoBuilder.followPath(doubleLoadingTrajectory[2]),
        SetCone(m_intake, true).ToPtr()
    );
}