// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CommandTestBase.h"
#include "frc/command2/WaitUntilCommand.h"

class WaitUntilCommandTest : public CommandTestBase {};

TEST_F(WaitUntilCommandTest, WaitUntilCommandSchedule) {
  frc::CommandScheduler scheduler = GetScheduler();

  bool finished = false;

  frc::WaitUntilCommand command([&finished] { return finished; });

  scheduler.Schedule(&command);
  scheduler.Run();
  EXPECT_TRUE(scheduler.IsScheduled(&command));
  finished = true;
  scheduler.Run();
  EXPECT_FALSE(scheduler.IsScheduled(&command));
}
