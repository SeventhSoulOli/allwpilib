// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/command2/CommandHelper.h>
#include <frc/command2/PIDCommand.h>

class ReplaceMePIDCommand
    : public frc::CommandHelper<frc::PIDCommand, ReplaceMePIDCommand> {
 public:
  ReplaceMePIDCommand();

  bool IsFinished() override;
};
