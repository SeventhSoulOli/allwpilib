// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <regex>

#include "CommandTestBase.h"
#include "frc/command2/PrintCommand.h"

class PrintCommandTest : public CommandTestBase {};

TEST_F(PrintCommandTest, PrintCommandSchedule) {
  frc::CommandScheduler scheduler = GetScheduler();

  frc::PrintCommand command("Test!");

  testing::internal::CaptureStdout();

  scheduler.Schedule(&command);
  scheduler.Run();

  EXPECT_TRUE(std::regex_search(testing::internal::GetCapturedStdout(),
                                std::regex("Test!")));

  EXPECT_FALSE(scheduler.IsScheduled(&command));
}
