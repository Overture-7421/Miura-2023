// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/XboxController.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Subsystems/SwerveChassis/SwerveChassis.h"
#include "Subsystems/VisionManager/VisionManager.h"
#include "Subsystems/DoubleArm/DoubleArm.h"
#include "Subsystems/Intake/Intake.h"

#include "Commands/Autonomous/CreateAuto.h"
#include "Commands/Teleop/Drive/Drive.h"
#include "Commands/Common/AlignRobotToTarget/AlignRobotToTarget.h"
#include "Commands/Common/UpdateVisionOdometry/UpdateVisionOdometry.h"
#include "Commands/Common/IntakeControl/IntakeControl.h"
#include "Commands/Common/AutoBalance/AutoBalance.h"

class RobotContainer {
public:
    RobotContainer();
    frc2::CommandPtr GetAutonomousCommand();
    void setVisionManager();

private:
    void ConfigureBindings();

    // Chassis driver controller and buttons
    frc::XboxController controller{ 0 };
    frc2::Trigger resetNavx{ [this] {return controller.GetBackButton();} };
    frc2::Trigger alignOneLeft{ [this] {return controller.GetLeftBumper() && controller.GetXButton();} };
    frc2::Trigger alignOneCenter{ [this] {return controller.GetLeftBumper() && controller.GetAButton();} };
    frc2::Trigger alignOneRight{ [this] {return controller.GetLeftBumper() && controller.GetBButton();} };
    frc2::Trigger alignTwoLeft{ [this] {return (controller.GetLeftTriggerAxis() || controller.GetRightTriggerAxis()) && controller.GetXButton();} };
    frc2::Trigger alignTwoCenter{ [this] {return (controller.GetLeftTriggerAxis() || controller.GetRightTriggerAxis()) && controller.GetAButton();} };
    frc2::Trigger alignTwoRight{ [this] {return (controller.GetLeftTriggerAxis() || controller.GetRightTriggerAxis()) && controller.GetBButton();} };
    frc2::Trigger alignThreeLeft{ [this] {return controller.GetRightBumper() && controller.GetXButton();} };
    frc2::Trigger alignThreeCenter{ [this] {return controller.GetRightBumper() && controller.GetAButton();} };
    frc2::Trigger alignThreeRight{ [this] {return controller.GetRightBumper() && controller.GetBButton();} };
    frc2::Trigger alignLoading{ [this] {return controller.GetYButton();} };
    frc2::Trigger autoBalance{ [this] {return controller.GetStartButton();} };

    // Mechanism Controller
    frc::XboxController mechanisms{ 1 };

    //Pneumatics
    frc2::Trigger conePiston{ [this] {return mechanisms.GetRightBumper();} };
    frc2::Trigger wristPiston{ [this] {return mechanisms.GetLeftBumper();} };

    //Positions
    frc2::Trigger upperPosition{ [this] {return mechanisms.GetYButton();} }; //Upper
    frc2::Trigger middlePosition{ [this] {return mechanisms.GetXButton();} }; //Middle
    frc2::Trigger groundPickUp{ [this] {return mechanisms.GetAButton();} }; //Ground
    frc2::Trigger portalPosition{ [this] {return mechanisms.GetBButton();} }; //Portal
    frc2::Trigger lowerPosition{ [this] {return mechanisms.GetPOV(0.75);} }; //Closed

    // Subsystems
    SwerveChassis swerveChassis;
    VisionManager visionManager{ &swerveChassis };
    Intake intake;
    DoubleArm doubleArm;

    // Auto
    CreateAuto createAuto{ &swerveChassis, &doubleArm, &intake };
};
