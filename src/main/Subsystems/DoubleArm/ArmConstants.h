#pragma once

#include<frc/trajectory/TrapezoidProfile.h>
#include<frc/geometry/Translation2d.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>


typedef frc::TrapezoidProfile<units::meters> PlannerProfile;
namespace ArmConstants {
    namespace Positions {
        const frc::Translation2d closed{ 0.04_m, 0.00_m };
        const frc::Translation2d upper{ 1_m, 0.65_m };
        const frc::Translation2d middle{ 0.55_m, 0.25_m };
        const frc::Translation2d ground{ 1.11_m, -.5_m };
        const frc::Translation2d portal{ 0.9_m, 0.47_m };
    };

    namespace Speeds {
        const PlannerProfile::Constraints middle{ 4.5_mps, 3.8_mps_sq };
        const PlannerProfile::Constraints closed{ 3.0_mps, 3.8_mps_sq };
        const PlannerProfile::Constraints upper{ 1.5_mps, 2_mps_sq };
        const PlannerProfile::Constraints ground{ 4.5_mps, 3.8_mps_sq };
        const PlannerProfile::Constraints portal{ 4.5_mps, 3.8_mps_sq };
    };

};