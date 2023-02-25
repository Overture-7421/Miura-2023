// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DoubleArmKinematics.h"
#include <units/length.h>
#include <algorithm>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

DoubleArmKinematics::DoubleArmKinematics(double lowerLength, double upperLength) {
    this->lowerLength = lowerLength;
    this->upperLength = upperLength;
}

frc::Translation2d DoubleArmKinematics::GetEndpointCoord(DoubleArmState currentState) {
    frc::Translation2d lowerSectionTrans = { units::meter_t(currentState.lowerAngle.Cos() * lowerLength) , units::meter_t(currentState.lowerAngle.Sin() * lowerLength) };
    frc::Translation2d upperSectionTrans = { units::meter_t(currentState.upperAngle.Cos() * upperLength) , units::meter_t(currentState.upperAngle.Sin() * upperLength) };
    return lowerSectionTrans + upperSectionTrans;
}

std::optional<DoubleArmState> DoubleArmKinematics::GetStateForTargetCoord(frc::Translation2d targetPoint) {
    //Using https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/

    double targetX = targetPoint.X().value();
    double targetY = targetPoint.Y().value();

    if (targetX == 0 && targetY == 0) {
        return {};
    }

    double targetUpperAngle;
    double targetLowerAngle;

    double cosParam = (std::pow(targetX, 2) + std::pow(targetY, 2) - std::pow(lowerLength, 2) - std::pow(upperLength, 2)) / (2.0 * lowerLength * upperLength);
    cosParam = std::clamp(cosParam, -1.0, 1.0);

    if (targetX >= 0) { //Going forward, need joint to bend downwards
        targetUpperAngle = std::acos(cosParam);
        targetLowerAngle = std::atan2(targetY, targetX) - std::atan2((upperLength * std::sin(targetUpperAngle)), (lowerLength + upperLength * std::cos(targetUpperAngle)));
    } else { //Goind backward, need joint to also bend downwards
        targetUpperAngle = -std::acos(cosParam);
        targetLowerAngle = std::atan2(targetY, targetX) + std::atan2((upperLength * std::sin(targetUpperAngle)), (lowerLength + upperLength * std::cos(targetUpperAngle)));
        targetUpperAngle *= -1;
    }


    DoubleArmState targetState;
    targetState.lowerAngle = units::radian_t(targetLowerAngle);
    targetState.upperAngle = units::radian_t(targetLowerAngle + targetUpperAngle);

    return targetState;
}

double DoubleArmKinematics::rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

double DoubleArmKinematics::deg2rad(double deg) {
    return deg * M_PI / 180.0;
}
