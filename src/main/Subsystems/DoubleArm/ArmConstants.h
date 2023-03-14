#pragma once

#include<frc/trajectory/TrapezoidProfile.h>
#include<frc/geometry/Translation2d.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>


typedef frc::TrapezoidProfile<units::meters> PlannerProfile;
namespace ArmConstants {
    namespace Positions {
        const frc::Translation2d closed{ 0.21_m, 0.05_m };
        const frc::Translation2d upper{ 1.21_m, 0.64_m };
        const frc::Translation2d middle{ 0.76_m, 0.22_m };
        const frc::Translation2d ground{ 1.08_m, -.5_m };
        const frc::Translation2d portal{ 0.82_m, 0.23_m };
    };

    namespace Speeds {
        const PlannerProfile::Constraints middle{ 4.5_mps, 3.8_mps_sq };
        const PlannerProfile::Constraints closed{ 4.0_mps, 4.5_mps_sq };
        const PlannerProfile::Constraints upper{ 1.5_mps, 2_mps_sq };
        const PlannerProfile::Constraints ground{ 4.5_mps, 3.8_mps_sq };
        const PlannerProfile::Constraints portal{ 4.5_mps, 3.8_mps_sq };
    };

};