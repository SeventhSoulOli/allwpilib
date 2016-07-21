/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SensorBase.h"

#include "FRC_NetworkCommunication/LoadOut.h"
#include "HAL/HAL.h"
#include "WPIErrors.h"

const int SensorBase::kDigitalChannels = HAL_GetNumDigitalPins();
const int SensorBase::kAnalogInputs = HAL_GetNumAnalogInputs();
const int SensorBase::kSolenoidChannels = HAL_GetNumSolenoidPins();
const int SensorBase::kSolenoidModules = HAL_GetNumPCMModules();
const int SensorBase::kPwmChannels = HAL_GetNumPWMPins();
const int SensorBase::kRelayChannels = HAL_GetNumRelayHeaders();
const int SensorBase::kPDPChannels = HAL_GetNumPDPChannels();

/**
 * Check that the solenoid module number is valid.
 *
 * @return Solenoid module is valid and present
 */
bool SensorBase::CheckSolenoidModule(int moduleNumber) {
  return HAL_CheckSolenoidModule(moduleNumber);
}

/**
 * Check that the digital channel number is valid.
 *
 * Verify that the channel number is one of the legal channel numbers. Channel
 * numbers are 1-based.
 *
 * @return Digital channel is valid
 */
bool SensorBase::CheckDigitalChannel(int channel) {
  return HAL_CheckDIOChannel(channel);
}

/**
 * Check that the relay channel number is valid.
 *
 * Verify that the channel number is one of the legal channel numbers. Channel
 * numbers are 0-based.
 *
 * @return Relay channel is valid
 */
bool SensorBase::CheckRelayChannel(int channel) {
  return HAL_CheckRelayChannel(channel);
}

/**
 * Check that the digital channel number is valid.
 *
 * Verify that the channel number is one of the legal channel numbers. Channel
 * numbers are 1-based.
 *
 * @return PWM channel is valid
 */
bool SensorBase::CheckPWMChannel(int channel) {
  return HAL_CheckPWMChannel(channel);
}

/**
 * Check that the analog input number is value.
 *
 * Verify that the analog input number is one of the legal channel numbers.
 * Channel numbers are 0-based.
 *
 * @return Analog channel is valid
 */
bool SensorBase::CheckAnalogInput(int channel) {
  return HAL_CheckAnalogInputChannel(channel);
}

/**
 * Check that the analog output number is valid.
 *
 * Verify that the analog output number is one of the legal channel numbers.
 * Channel numbers are 0-based.
 *
 * @return Analog channel is valid
 */
bool SensorBase::CheckAnalogOutput(int channel) {
  return HAL_CheckAnalogOutputChannel(channel);
}

/**
 * Verify that the solenoid channel number is within limits.
 *
 * @return Solenoid channel is valid
 */
bool SensorBase::CheckSolenoidChannel(int channel) {
  return HAL_CheckSolenoidChannel(channel);
}

/**
 * Verify that the power distribution channel number is within limits.
 *
 * @return PDP channel is valid
 */
bool SensorBase::CheckPDPChannel(int channel) {
  return HAL_CheckPDPModule(channel);
}
