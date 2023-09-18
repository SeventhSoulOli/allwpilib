// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/command2/Command.h>
#include <frc/command2/CommandHelper.h>

#include "subsystems/Elevator.h"

/**
 * Move the elevator to a given location. This command finishes when it is
 * within
 * the tolerance, but leaves the PID loop running to maintain the position.
 * Other
 * commands using the elevator should make sure they disable PID!
 */
class SetElevatorSetpoint
    : public frc::CommandHelper<frc::Command, SetElevatorSetpoint> {
 public:
  explicit SetElevatorSetpoint(double setpoint, Elevator& elevator);
  void Initialize() override;
  bool IsFinished() override;

 private:
  double m_setpoint;
  Elevator* m_elevator;
};
