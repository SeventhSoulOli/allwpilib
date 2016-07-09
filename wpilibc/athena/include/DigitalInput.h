/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>

#include <memory>

#include "DigitalSource.h"
#include "LiveWindow/LiveWindowSendable.h"

class DigitalGlitchFilter;

/**
 * Class to read a digital input.
 * This class will read digital inputs and return the current value on the
 * channel. Other devices such as encoders, gear tooth sensors, etc. that are
 * implemented elsewhere will automatically allocate digital inputs and outputs
 * as required. This class is only for devices like switches etc. that aren't
 * implemented anywhere else.
 */
class DigitalInput : public DigitalSource, public LiveWindowSendable {
 public:
  explicit DigitalInput(uint32_t channel);
  virtual ~DigitalInput();
  bool Get() const;
  uint32_t GetChannel() const override;

  // Digital Source Interface
  virtual HAL_Handle GetPortHandleForRouting() const override;
  virtual AnalogTriggerType GetAnalogTriggerTypeForRouting() const override;
  virtual bool IsAnalogTrigger() const override;

  void UpdateTable();
  void StartLiveWindowMode();
  void StopLiveWindowMode();
  std::string GetSmartDashboardType() const;
  void InitTable(std::shared_ptr<ITable> subTable);
  std::shared_ptr<ITable> GetTable() const;

 private:
  uint32_t m_channel;
  HAL_DigitalHandle m_handle;

  std::shared_ptr<ITable> m_table;
  friend class DigitalGlitchFilter;
};
