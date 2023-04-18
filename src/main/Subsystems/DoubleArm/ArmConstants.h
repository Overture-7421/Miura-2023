#pragma once

#include<frc/trajectory/TrapezoidProfile.h>
#include<frc/geometry/Translation2d.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>



typedef frc::TrapezoidProfile<units::meters> PlannerProfile;
namespace ArmConstants {
    namespace Positions {
        const frc::Translation2d closed{ 0.11_m, 0.02_m };
        const frc::Translation2d upper{ 1_m, 0.80_m };
        const frc::Translation2d middle{ 0.55_m, 0.22_m };
        const frc::Translation2d ground{ 1.02_m, -.54_m };
        const frc::Translation2d portal{ 0.87_m, 0.38_m };

        const frc::Translation2d armInvertedAuto{ 0.10_m, 0.20_m };
        const frc::Translation2d closedauto{ 0.11_m, 0.02_m };
        const frc::Translation2d groundAuto{ 1.02_m, -.58_m };
        const frc::Translation2d portalAuto{ 0.80_m, 0.34_m };
    };

    namespace Speeds {
        const PlannerProfile::Constraints middle{ 4.5_mps, 3.8_mps_sq };
        const PlannerProfile::Constraints closed{ 4.5_mps, 3.8_mps_sq };
        const PlannerProfile::Constraints upper{ 1.5_mps, 2_mps_sq };
        const PlannerProfile::Constraints ground{ 4.5_mps, 3.8_mps_sq };
        const PlannerProfile::Constraints portal{ 4.5_mps, 3.8_mps_sq };

        const PlannerProfile::Constraints armInvertedAuto{ 1_mps, 1_mps_sq };
        const PlannerProfile::Constraints closedauto{ 1_mps, 1_mps_sq };
        const PlannerProfile::Constraints groundAuto{ 4.5_mps, 3.8_mps_sq };

    };

    namespace AutoPieces {
        const double AutoTopCube = 7.5;
        const double AutoMiddleCube = 4.4;

    };
};