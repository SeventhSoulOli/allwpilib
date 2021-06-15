// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/IterativeRobotBase.h"

#include <fmt/format.h>
#include <hal/DriverStation.h>
#include <networktables/NetworkTableInstance.h>

#include "frc/Errors.h"
#include "frc/livewindow/LiveWindow.h"
#include "frc/shuffleboard/Shuffleboard.h"
#include "frc/smartdashboard/SmartDashboard.h"

using namespace frc;

IterativeRobotBase::IterativeRobotBase(double period)
    : IterativeRobotBase(units::second_t(period)) {}

IterativeRobotBase::IterativeRobotBase(units::second_t period)
    : m_period(period),
      m_watchdog(period, [this] { PrintLoopOverrunMessage(); }) {}

void IterativeRobotBase::RobotInit() {
  fmt::print("Default {}() method... Override me!\n", __FUNCTION__);
}

void IterativeRobotBase::SimulationInit() {
  fmt::print("Default {}() method... Override me!\n", __FUNCTION__);
}

void IterativeRobotBase::DisabledInit() {
  fmt::print("Default {}() method... Override me!\n", __FUNCTION__);
}

void IterativeRobotBase::AutonomousInit() {
  fmt::print("Default {}() method... Override me!\n", __FUNCTION__);
}

void IterativeRobotBase::TeleopInit() {
  fmt::print("Default {}() method... Override me!\n", __FUNCTION__);
}

void IterativeRobotBase::TestInit() {
  fmt::print("Default {}() method... Override me!\n", __FUNCTION__);
}

void IterativeRobotBase::RobotPeriodic() {
  static bool firstRun = true;
  if (firstRun) {
    fmt::print("Default {}() method... Override me!\n", __FUNCTION__);
    firstRun = false;
  }
}

void IterativeRobotBase::SimulationPeriodic() {
  static bool firstRun = true;
  if (firstRun) {
    fmt::print("Default {}() method... Override me!\n", __FUNCTION__);
    firstRun = false;
  }
}

void IterativeRobotBase::DisabledPeriodic() {
  static bool firstRun = true;
  if (firstRun) {
    fmt::print("Default {}() method... Override me!\n", __FUNCTION__);
    firstRun = false;
  }
}

void IterativeRobotBase::AutonomousPeriodic() {
  static bool firstRun = true;
  if (firstRun) {
    fmt::print("Default {}() method... Override me!\n", __FUNCTION__);
    firstRun = false;
  }
}

void IterativeRobotBase::TeleopPeriodic() {
  static bool firstRun = true;
  if (firstRun) {
    fmt::print("Default {}() method... Override me!\n", __FUNCTION__);
    firstRun = false;
  }
}

void IterativeRobotBase::TestPeriodic() {
  static bool firstRun = true;
  if (firstRun) {
    fmt::print("Default {}() method... Override me!\n", __FUNCTION__);
    firstRun = false;
  }
}

void IterativeRobotBase::SetNetworkTablesFlushEnabled(bool enabled) {
  m_ntFlushEnabled = enabled;
}

units::second_t IterativeRobotBase::GetPeriod() const {
  return m_period;
}

void IterativeRobotBase::LoopFunc() {
  m_watchdog.Reset();

  // Call the appropriate function depending upon the current robot mode
  if (IsDisabled()) {
    // Call DisabledInit() if we are now just entering disabled mode from
    // either a different mode or from power-on.
    if (m_lastMode != Mode::kDisabled) {
      LiveWindow::SetEnabled(false);
      Shuffleboard::DisableActuatorWidgets();
      DisabledInit();
      m_watchdog.AddEpoch("DisabledInit()");
      m_lastMode = Mode::kDisabled;
    }

    HAL_ObserveUserProgramDisabled();
    DisabledPeriodic();
    m_watchdog.AddEpoch("DisabledPeriodic()");
  } else if (IsAutonomous()) {
    // Call AutonomousInit() if we are now just entering autonomous mode from
    // either a different mode or from power-on.
    if (m_lastMode != Mode::kAutonomous) {
      LiveWindow::SetEnabled(false);
      Shuffleboard::DisableActuatorWidgets();
      AutonomousInit();
      m_watchdog.AddEpoch("AutonomousInit()");
      m_lastMode = Mode::kAutonomous;
    }

    HAL_ObserveUserProgramAutonomous();
    AutonomousPeriodic();
    m_watchdog.AddEpoch("AutonomousPeriodic()");
  } else if (IsOperatorControl()) {
    // Call TeleopInit() if we are now just entering teleop mode from
    // either a different mode or from power-on.
    if (m_lastMode != Mode::kTeleop) {
      LiveWindow::SetEnabled(false);
      Shuffleboard::DisableActuatorWidgets();
      TeleopInit();
      m_watchdog.AddEpoch("TeleopInit()");
      m_lastMode = Mode::kTeleop;
    }

    HAL_ObserveUserProgramTeleop();
    TeleopPeriodic();
    m_watchdog.AddEpoch("TeleopPeriodic()");
  } else {
    // Call TestInit() if we are now just entering test mode from
    // either a different mode or from power-on.
    if (m_lastMode != Mode::kTest) {
      LiveWindow::SetEnabled(true);
      Shuffleboard::EnableActuatorWidgets();
      TestInit();
      m_watchdog.AddEpoch("TestInit()");
      m_lastMode = Mode::kTest;
    }

    HAL_ObserveUserProgramTest();
    TestPeriodic();
    m_watchdog.AddEpoch("TestPeriodic()");
  }

  RobotPeriodic();
  m_watchdog.AddEpoch("RobotPeriodic()");

  SmartDashboard::UpdateValues();
  m_watchdog.AddEpoch("SmartDashboard::UpdateValues()");
  LiveWindow::UpdateValues();
  m_watchdog.AddEpoch("LiveWindow::UpdateValues()");
  Shuffleboard::Update();
  m_watchdog.AddEpoch("Shuffleboard::Update()");

  if constexpr (IsSimulation()) {
    HAL_SimPeriodicBefore();
    SimulationPeriodic();
    HAL_SimPeriodicAfter();
    m_watchdog.AddEpoch("SimulationPeriodic()");
  }

  m_watchdog.Disable();

  // Flush NetworkTables
  if (m_ntFlushEnabled) {
    nt::NetworkTableInstance::GetDefault().Flush();
  }

  // Warn on loop time overruns
  if (m_watchdog.IsExpired()) {
    m_watchdog.PrintEpochs();
  }
}

void IterativeRobotBase::PrintLoopOverrunMessage() {
  FRC_ReportError(err::Error, "Loop time of {:.6f}s overrun",
                  m_period.to<double>());
}
