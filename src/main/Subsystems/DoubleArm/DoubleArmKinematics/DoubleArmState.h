#pragma once
#include <frc/geometry/Rotation2d.h>

struct DoubleArmState {
    frc::Rotation2d upperAngle; //  Angle of lower joint, relative to X axis
    frc::Rotation2d lowerAngle; //  Angle of upper joint, relative to X axis
};