// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/command2/CommandHelper.h>
#include <frc/command2/PIDCommand.h>

#include "subsystems/Drivetrain.h"

/**
 * Drive until the robot is the given distance away from the box. Uses a local
 * PID controller to run a simple PID loop that is only enabled while this
 * command is running. The input is the averaged values of the left and right
 * encoders.
 */
class SetDistanceToBox
    : public frc::CommandHelper<frc::PIDCommand, SetDistanceToBox> {
 public:
  explicit SetDistanceToBox(double distance, Drivetrain& drivetrain);
  void Initialize() override;
  bool IsFinished() override;

 private:
  Drivetrain* m_drivetrain;
};
