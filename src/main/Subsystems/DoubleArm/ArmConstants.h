#pragma once

#include<frc/trajectory/TrapezoidProfile.h>
#include<frc/geometry/Translation2d.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include "Subsystems/DoubleArm/DoubleArmKinematics/DoubleArmState.h"

namespace ArmConstants {
    namespace Positions {
        const DoubleArmState StartingPosition{ 101_deg, -71_deg };
        const DoubleArmState MiddlePosition{ 0_deg, 90_deg };

    };

    namespace AutoPieces {
        const double AutoTopCube = 7.5;
        const double AutoMiddleCube = 4.4;

    };
};