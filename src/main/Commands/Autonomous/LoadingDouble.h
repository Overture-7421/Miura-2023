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

#include <Subsystems/DoubleArm/ArmConstants.h>

using namespace ArmConstants;

static frc2::CommandPtr LoadingDouble(SwerveChassis* m_swerveChassis, DoubleArm* m_doubleArm, Intake* m_intake, pathplanner::SwerveAutoBuilder* autoBuilder) {
    std::vector<pathplanner::PathPlannerTrajectory> doubleLoadingTrajectory = pathplanner::PathPlanner::loadPathGroup("LoadingDouble", { pathplanner::PathConstraints(2_mps, 2_mps_sq) });


    return frc2::cmd::Sequence(
        frc2::InstantCommand([m_swerveChassis = m_swerveChassis]() {m_swerveChassis->resetOdometry({ 1.8_m, 0.5_m, {180_deg} });}).ToPtr(),
        SetWrist(m_intake, false).ToPtr(),
        SetArmCoordinate(m_doubleArm, Positions::middle, Speeds::middle).ToPtr(), //Middle
        SetCone(m_intake, true).ToPtr(),
        frc2::WaitCommand{ 0.5_s }.ToPtr(),
        SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr(), //Closed
        autoBuilder->followPath(doubleLoadingTrajectory[0]),
        SetWrist(m_intake, true).ToPtr(),
        SetCone(m_intake, true).ToPtr(),
        SetArmCoordinate(m_doubleArm, Positions::ground, Speeds::ground).ToPtr(), //Ground
        autoBuilder->followPath(doubleLoadingTrajectory[1]),
        SetCone(m_intake, false).ToPtr(),
        frc2::WaitCommand{ 0.5_s }.ToPtr(),
        SetArmCoordinate(m_doubleArm, Positions::closed, Speeds::closed).ToPtr(), //Closed
        SetWrist(m_intake, false).ToPtr(),
        autoBuilder->followPath(doubleLoadingTrajectory[2]),
        SetCone(m_intake, true).ToPtr()
    );
}