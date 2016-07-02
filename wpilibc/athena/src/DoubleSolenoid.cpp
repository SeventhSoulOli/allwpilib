/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "DoubleSolenoid.h"
#include "HAL/HAL.h"
#include "LiveWindow/LiveWindow.h"
#include "WPIErrors.h"

#include <sstream>

/**
 * Constructor.
 *
 * Uses the default PCM ID of 0.
 *
 * @param forwardChannel The forward channel number on the PCM (0..7).
 * @param reverseChannel The reverse channel number on the PCM (0..7).
 */
DoubleSolenoid::DoubleSolenoid(uint32_t forwardChannel, uint32_t reverseChannel)
    : DoubleSolenoid(GetDefaultSolenoidModule(), forwardChannel,
                     reverseChannel) {}

/**
 * Constructor.
 *
 * @param moduleNumber   The CAN ID of the PCM.
 * @param forwardChannel The forward channel on the PCM to control (0..7).
 * @param reverseChannel The reverse channel on the PCM to control (0..7).
 */
DoubleSolenoid::DoubleSolenoid(uint8_t moduleNumber, uint32_t forwardChannel,
                               uint32_t reverseChannel)
    : SolenoidBase(moduleNumber),
      m_forwardChannel(forwardChannel),
      m_reverseChannel(reverseChannel) {
  std::stringstream buf;
  if (!CheckSolenoidModule(m_moduleNumber)) {
    buf << "Solenoid Module " << m_moduleNumber;
    wpi_setWPIErrorWithContext(ModuleIndexOutOfRange, buf.str());
    return;
  }
  if (!CheckSolenoidChannel(m_forwardChannel)) {
    buf << "Solenoid Module " << m_forwardChannel;
    wpi_setWPIErrorWithContext(ChannelIndexOutOfRange, buf.str());
    return;
  }
  if (!CheckSolenoidChannel(m_reverseChannel)) {
    buf << "Solenoid Module " << m_reverseChannel;
    wpi_setWPIErrorWithContext(ChannelIndexOutOfRange, buf.str());
    return;
  }
  int32_t status = 0;
  m_forwardHandle = initializeSolenoidPort(
      getPortWithModule(moduleNumber, m_forwardChannel), &status);
  if (status != 0) {
    wpi_setErrorWithContext(status, getHALErrorMessage(status));
    m_forwardHandle = HAL_INVALID_HANDLE;
    m_reverseHandle = HAL_INVALID_HANDLE;
    return;
  }

  m_reverseHandle = initializeSolenoidPort(
      getPortWithModule(moduleNumber, m_reverseChannel), &status);
  if (status != 0) {
    wpi_setErrorWithContext(status, getHALErrorMessage(status));
    // free forward solenoid
    freeSolenoidPort(m_forwardHandle);
    m_forwardHandle = HAL_INVALID_HANDLE;
    m_reverseHandle = HAL_INVALID_HANDLE;
    return;
  }

  m_forwardMask = 1 << m_forwardChannel;
  m_reverseMask = 1 << m_reverseChannel;

  HALReport(HALUsageReporting::kResourceType_Solenoid, m_forwardChannel,
            m_moduleNumber);
  HALReport(HALUsageReporting::kResourceType_Solenoid, m_reverseChannel,
            m_moduleNumber);
  LiveWindow::GetInstance()->AddActuator("DoubleSolenoid", m_moduleNumber,
                                         m_forwardChannel, this);
}

/**
 * Destructor.
 */
DoubleSolenoid::~DoubleSolenoid() {
  freeSolenoidPort(m_forwardHandle);
  freeSolenoidPort(m_reverseHandle);
  if (m_table != nullptr) m_table->RemoveTableListener(this);
}

/**
 * Set the value of a solenoid.
 *
 * @param value The value to set (Off, Forward or Reverse)
 */
void DoubleSolenoid::Set(Value value) {
  if (StatusIsFatal()) return;

  bool forward = false;
  bool reverse = false;
  switch (value) {
    case kOff:
      forward = false;
      reverse = false;
      break;
    case kForward:
      forward = true;
      reverse = false;
      break;
    case kReverse:
      forward = false;
      reverse = true;
      break;
  }
  int32_t fstatus = 0;
  setSolenoid(m_forwardHandle, forward, &fstatus);
  int32_t rstatus = 0;
  setSolenoid(m_reverseHandle, reverse, &rstatus);

  wpi_setErrorWithContext(fstatus, getHALErrorMessage(fstatus));
  wpi_setErrorWithContext(rstatus, getHALErrorMessage(rstatus));
}

/**
 * Read the current value of the solenoid.
 *
 * @return The current value of the solenoid.
 */
DoubleSolenoid::Value DoubleSolenoid::Get() const {
  if (StatusIsFatal()) return kOff;
  int32_t fstatus = 0;
  int32_t rstatus = 0;
  bool valueForward = getSolenoid(m_forwardHandle, &fstatus);
  bool valueReverse = getSolenoid(m_reverseHandle, &rstatus);

  wpi_setErrorWithContext(fstatus, getHALErrorMessage(fstatus));
  wpi_setErrorWithContext(rstatus, getHALErrorMessage(rstatus));

  if (valueForward) return kForward;
  if (valueReverse) return kReverse;
  return kOff;
}
/**
 * Check if the forward solenoid is blacklisted.
 *
 * If a solenoid is shorted, it is added to the blacklist and
 * disabled until power cycle, or until faults are cleared.
 * @see ClearAllPCMStickyFaults()
 *
 * @return If solenoid is disabled due to short.
 */
bool DoubleSolenoid::IsFwdSolenoidBlackListed() const {
  int blackList = GetPCMSolenoidBlackList(m_moduleNumber);
  return (blackList & m_forwardMask) ? 1 : 0;
}
/**
 * Check if the reverse solenoid is blacklisted.
 *
 * If a solenoid is shorted, it is added to the blacklist and
 * disabled until power cycle, or until faults are cleared.
 * @see ClearAllPCMStickyFaults()
 *
 * @return If solenoid is disabled due to short.
 */
bool DoubleSolenoid::IsRevSolenoidBlackListed() const {
  int blackList = GetPCMSolenoidBlackList(m_moduleNumber);
  return (blackList & m_reverseMask) ? 1 : 0;
}

void DoubleSolenoid::ValueChanged(ITable* source, llvm::StringRef key,
                                  std::shared_ptr<nt::Value> value,
                                  bool isNew) {
  if (!value->IsString()) return;
  Value lvalue = kOff;
  if (value->GetString() == "Forward")
    lvalue = kForward;
  else if (value->GetString() == "Reverse")
    lvalue = kReverse;
  Set(lvalue);
}

void DoubleSolenoid::UpdateTable() {
  if (m_table != nullptr) {
    m_table->PutString(
        "Value", (Get() == kForward ? "Forward"
                                    : (Get() == kReverse ? "Reverse" : "Off")));
  }
}

void DoubleSolenoid::StartLiveWindowMode() {
  Set(kOff);
  if (m_table != nullptr) {
    m_table->AddTableListener("Value", this, true);
  }
}

void DoubleSolenoid::StopLiveWindowMode() {
  Set(kOff);
  if (m_table != nullptr) {
    m_table->RemoveTableListener(this);
  }
}

std::string DoubleSolenoid::GetSmartDashboardType() const {
  return "Double Solenoid";
}

void DoubleSolenoid::InitTable(std::shared_ptr<ITable> subTable) {
  m_table = subTable;
  UpdateTable();
}

std::shared_ptr<ITable> DoubleSolenoid::GetTable() const { return m_table; }
