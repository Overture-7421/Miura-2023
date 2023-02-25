// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/geometry/Translation2d.h>
#include <optional>

#include "DoubleArmState.h"

class DoubleArmKinematics {
public:
    DoubleArmKinematics(double lowerLength, double upperLength);

    frc::Translation2d GetEndpointCoord(DoubleArmState currentState);

    std::optional<DoubleArmState> GetStateForTargetCoord(frc::Translation2d targetPoint);

private:

    double rad2deg(double rad);
    double deg2rad(double deg);

    double lowerLength; // Length in meters of lower section
    double upperLength; // Length in meters of upper section

};


class ZeroCoordException: public std::exception {
public:
    const char* what() {
        return "Invalid target coord requested to DoubleArmKinematics! X and Y cant be both 0...";
    }
};
