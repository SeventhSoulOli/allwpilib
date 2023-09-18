// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/command2/CommandHelper.h>
#include <frc/command2/SequentialCommandGroup.h>

#include "subsystems/Claw.h"
#include "subsystems/Elevator.h"
#include "subsystems/Wrist.h"

/**
 * Place a held soda can onto the platform.
 */
class Place : public frc::CommandHelper<frc::SequentialCommandGroup, Place> {
 public:
  Place(Claw& claw, Wrist& wrist, Elevator& elevator);
};
