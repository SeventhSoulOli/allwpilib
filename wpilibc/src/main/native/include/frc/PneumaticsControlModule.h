// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <hal/Types.h>

#include "PneumaticsBase.h"

namespace frc {
class PneumaticsControlModule : public PneumaticsBase {
 public:
  PneumaticsControlModule();
  explicit PneumaticsControlModule(int module);

  ~PneumaticsControlModule() override;

  PneumaticsControlModule(PneumaticsControlModule&&) = default;
  PneumaticsControlModule& operator=(PneumaticsControlModule&&) = default;

  bool GetCompressor();

  void SetClosedLoopControl(bool enabled);

  bool GetClosedLoopControl();

  bool GetPressureSwitch();

  double GetCompressorCurrent();

  bool GetCompressorCurrentTooHighFault();
  bool GetCompressorCurrentTooHighStickyFault();
  bool GetCompressorShortedFault();
  bool GetCompressorShortedStickyFault();
  bool GetCompressorNotConnectedFault();
  bool GetCompressorNotConnectedStickyFault();

  bool GetSolenoidVoltageFault();
  bool GetSolenoidVoltageStickyFault();

  void ClearAllStickyFaults();

  void SetSolenoids(int mask, int values) override;

  int GetSolenoids() const override;

  int GetModuleNumber() const override;

  int GetSolenoidDisabledList() const override;

  void FireOneShot(int index) override;

  void SetOneShotDuration(int index, units::second_t duration) override;

  bool CheckSolenoidChannel(int channel) const override;

 private:
  int m_module;
  hal::Handle<HAL_CTREPCMHandle> m_handle;
};
}  // namespace frc
