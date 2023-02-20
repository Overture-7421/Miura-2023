// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DoubleArmKinematics.h"
#include <units/length.h>

DoubleArmKinematics::DoubleArmKinematics(double lowerLength, double upperLength) {
    this->lowerLength = lowerLength;
    this->upperLength = upperLength;
}

frc::Translation2d DoubleArmKinematics::GetEndpointCoord(DoubleArmState currentState) {
    frc::Translation2d lowerSectionTrans = { units::meter_t(std::cos(deg2rad(currentState.lowerAngle)) * lowerLength) , units::meter_t(std::sin(deg2rad(currentState.lowerAngle) * lowerLength)) };
    frc::Translation2d upperSectionTrans = { units::meter_t(std::cos(deg2rad(currentState.upperAngle)) * upperLength) , units::meter_t(std::sin(deg2rad(currentState.upperAngle) * upperLength)) };
    return lowerSectionTrans + upperSectionTrans;
}

DoubleArmState DoubleArmKinematics::GetStateForTargetCoord(frc::Translation2d targetPoint) {
    //Using https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/

    double targetX = targetPoint.X().value();
    double targetY = targetPoint.Y().value();

    if (targetX == 0 && targetY == 0) {
        throw ZeroCoordException();
    }

    double targetUpperAngle;
    double targetLowerAngle;

    if (targetX >= 0) { //Going forward, need joint to bend downwards
        targetUpperAngle = acos((pow(targetX, 2) + pow(targetY, 2) - pow(lowerLength, 2) - pow(upperLength, 2)) / (2 * lowerLength * upperLength));
        targetLowerAngle = atan2(targetY, targetX) - atan2((upperLength * sin(targetUpperAngle)), (lowerLength + upperLength * cos(targetUpperAngle)));
    } else if (targetX < 0) { //Goind backward, need joint to also bend downwards
        targetUpperAngle = -acos((pow(targetX, 2) + pow(targetY, 2) - pow(lowerLength, 2) - pow(upperLength, 2)) / (2 * lowerLength * upperLength));
        targetLowerAngle = atan2(targetY, targetX) + atan2((upperLength * sin(targetUpperAngle)), (lowerLength + upperLength * cos(targetUpperAngle)));
    }


    DoubleArmState targetState;
    targetState.lowerAngle = targetLowerAngle;
    targetState.upperAngle = targetLowerAngle + targetUpperAngle;

    return targetState;
}

double DoubleArmKinematics::rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

double DoubleArmKinematics::deg2rad(double deg) {
    return deg * M_PI / 180.0;
}
