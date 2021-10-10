// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "hal/REVPH.h"

#include <fmt/format.h>

#include "HALInitializer.h"
#include "HALInternal.h"
#include "PortsInternal.h"
#include "hal/CANAPI.h"
#include "hal/Errors.h"
#include "hal/handles/IndexedHandleResource.h"
#include "rev/PHFrames.h"

using namespace hal;

static constexpr HAL_CANManufacturer manufacturer =
    HAL_CANManufacturer::HAL_CAN_Man_kREV;

static constexpr HAL_CANDeviceType deviceType =
    HAL_CANDeviceType::HAL_CAN_Dev_kPneumatics;

static constexpr int32_t kDefaultControlPeriod = 20;
// static constexpr uint8_t kDefaultSensorMask = (1 <<
// HAL_REV_PHSENSOR_DIGITAL);
static constexpr uint8_t kDefaultCompressorDuty = 255;
static constexpr uint8_t kDefaultPressureTarget = 120;
static constexpr uint8_t kDefaultPressureHysteresis = 60;

#define HAL_REV_MAX_PULSE_TIME 65534
#define HAL_REV_MAX_PRESSURE_TARGET 120
#define HAL_REV_MAX_PRESSURE_HYSTERESIS HAL_REV_MAX_PRESSURE_TARGET

static constexpr uint32_t APIFromExtId(uint32_t extId) {
  return (extId >> 6) & 0x3FF;
}

static constexpr uint32_t PH_SET_ALL_FRAME_API =
    APIFromExtId(PH_SET_ALL_FRAME_ID);
static constexpr uint32_t PH_PULSE_ONCE_FRAME_API =
    APIFromExtId(PH_PULSE_ONCE_FRAME_ID);
static constexpr uint32_t PH_STATUS0_FRAME_API =
    APIFromExtId(PH_STATUS0_FRAME_ID);
static constexpr uint32_t PH_STATUS1_FRAME_API =
    APIFromExtId(PH_STATUS1_FRAME_ID);

static constexpr int32_t kPHFrameStatus0Timeout = 50;
static constexpr int32_t kPHFrameStatus1Timeout = 50;

namespace {

struct REV_PHObj {
  int32_t controlPeriod;
  PH_set_all_t desiredSolenoidsState;
  wpi::mutex solenoidLock;
  HAL_CANHandle hcan;
  std::string previousAllocation;
};

}  // namespace

static IndexedHandleResource<HAL_REVPHHandle, REV_PHObj, 63,
                             HAL_HandleEnum::REVPH>* REVPHHandles;

namespace hal::init {
void InitializeREVPH() {
  static IndexedHandleResource<HAL_REVPHHandle, REV_PHObj, kNumREVPHModules,
                               HAL_HandleEnum::REVPH>
      rH;
  REVPHHandles = &rH;
}
}  // namespace hal::init

static PH_status0_t HAL_REV_ReadPHStatus0(HAL_CANHandle hcan, int32_t* status) {
  uint8_t packedData[8] = {0};
  int32_t length = 0;
  uint64_t timestamp = 0;
  PH_status0_t result = {};

  HAL_ReadCANPacketTimeout(hcan, PH_STATUS0_FRAME_API, packedData, &length,
                           &timestamp, kPHFrameStatus0Timeout * 2, status);

  if (*status != 0) {
    return result;
  }

  PH_status0_unpack(&result, packedData, PH_STATUS0_LENGTH);

  return result;
}

static PH_status1_t HAL_REV_ReadPHStatus1(HAL_CANHandle hcan, int32_t* status) {
  uint8_t packedData[8] = {0};
  int32_t length = 0;
  uint64_t timestamp = 0;
  PH_status1_t result = {};

  HAL_ReadCANPacketTimeout(hcan, PH_STATUS1_FRAME_API, packedData, &length,
                           &timestamp, kPHFrameStatus1Timeout * 2, status);

  if (*status != 0) {
    return result;
  }

  PH_status1_unpack(&result, packedData, PH_STATUS1_LENGTH);

  return result;
}

enum REV_SolenoidState {
  kSolenoidDisabled = 0,
  kSolenoidEnabled,
  kSolenoidControlledViaPulse
};

static void HAL_REV_UpdateDesiredPHSolenoidState(REV_PHObj* hph,
                                                 int32_t solenoid,
                                                 REV_SolenoidState state) {
  switch (solenoid) {
    case 0:
      hph->desiredSolenoidsState.channel_0 = state;
      break;
    case 1:
      hph->desiredSolenoidsState.channel_1 = state;
      break;
    case 2:
      hph->desiredSolenoidsState.channel_2 = state;
      break;
    case 3:
      hph->desiredSolenoidsState.channel_3 = state;
      break;
    case 4:
      hph->desiredSolenoidsState.channel_4 = state;
      break;
    case 5:
      hph->desiredSolenoidsState.channel_5 = state;
      break;
    case 6:
      hph->desiredSolenoidsState.channel_6 = state;
      break;
    case 7:
      hph->desiredSolenoidsState.channel_7 = state;
      break;
    case 8:
      hph->desiredSolenoidsState.channel_8 = state;
      break;
    case 9:
      hph->desiredSolenoidsState.channel_9 = state;
      break;
    case 10:
      hph->desiredSolenoidsState.channel_10 = state;
      break;
    case 11:
      hph->desiredSolenoidsState.channel_11 = state;
      break;
    case 12:
      hph->desiredSolenoidsState.channel_12 = state;
      break;
    case 13:
      hph->desiredSolenoidsState.channel_13 = state;
      break;
    case 14:
      hph->desiredSolenoidsState.channel_14 = state;
      break;
    case 15:
      hph->desiredSolenoidsState.channel_15 = state;
      break;
  }
}

static void HAL_REV_SendSolenoidsState(REV_PHObj* hph, int32_t* status) {
  uint8_t packedData[PH_SET_ALL_LENGTH] = {0};
  PH_set_all_pack(packedData, &(hph->desiredSolenoidsState), PH_SET_ALL_LENGTH);
  HAL_WriteCANPacketRepeating(hph->hcan, packedData, PH_SET_ALL_LENGTH,
                              PH_SET_ALL_FRAME_API, hph->controlPeriod, status);
}

static HAL_Bool HAL_REV_CheckPHPulseTime(int32_t time) {
  return ((time > 0) && (time <= HAL_REV_MAX_PULSE_TIME)) ? 1 : 0;
}

HAL_REVPHHandle HAL_InitializeREVPH(int32_t module,
                                    const char* allocationLocation,
                                    int32_t* status) {
  hal::init::CheckInit();
  if (!HAL_CheckREVPHModuleNumber(module)) {
    hal::SetLastErrorIndexOutOfRange(status, "Invalid Index for REV PH", 1,
                                     kNumREVPHModules, module);
    return HAL_kInvalidHandle;
  }

  HAL_REVPHHandle handle;
  auto hph = REVPHHandles->Allocate(module, &handle, status);
  if (*status != 0) {
    if (hph) {
      hal::SetLastErrorPreviouslyAllocated(status, "REV PH", module,
                                           hph->previousAllocation);
    } else {
      hal::SetLastErrorIndexOutOfRange(status, "Invalid Index for REV PH", 1,
                                       kNumREVPHModules, module);
    }
    return HAL_kInvalidHandle;  // failed to allocate. Pass error back.
  }

  HAL_CANHandle hcan =
      HAL_InitializeCAN(manufacturer, module, deviceType, status);

  if (*status != 0) {
    REVPHHandles->Free(handle);
    return HAL_kInvalidHandle;
  }

  hph->previousAllocation = allocationLocation ? allocationLocation : "";
  hph->hcan = hcan;
  hph->controlPeriod = kDefaultControlPeriod;

  // TODO any other things

  return handle;
}

void HAL_FreeREVPH(HAL_REVPHHandle handle) {
  auto hph = REVPHHandles->Get(handle);
  if (hph == nullptr)
    return;

  HAL_CleanCAN(hph->hcan);

  REVPHHandles->Free(handle);
}

HAL_Bool HAL_CheckREVPHModuleNumber(int32_t module) {
  return module >= 1 && module < kNumREVPDHModules;
}

HAL_Bool HAL_CheckREVPHSolenoidChannel(int32_t channel) {
  return channel >= 0 && channel < kNumREVPHChannels;
}

HAL_Bool HAL_GetREVPHCompressor(HAL_REVPHHandle handle, int32_t* status) {
  auto ph = REVPHHandles->Get(handle);
  if (ph == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return false;
  }

  PH_status0_t status0 = HAL_REV_ReadPHStatus0(ph->hcan, status);

  if (*status != 0) {
    return false;
  }

  return status0.compressor_on;
}

void HAL_SetREVPHClosedLoopControl(HAL_REVPHHandle handle, HAL_Bool enabled,
                                   int32_t* status) {
  // TODO
}

HAL_Bool HAL_GetREVPHClosedLoopControl(HAL_REVPHHandle handle,
                                       int32_t* status) {
  return false;  // TODO
}

HAL_Bool HAL_GetREVPHPressureSwitch(HAL_REVPHHandle handle, int32_t* status) {
  auto ph = REVPHHandles->Get(handle);
  if (ph == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return false;
  }

  PH_status0_t status0 = HAL_REV_ReadPHStatus0(ph->hcan, status);

  if (*status != 0) {
    return false;
  }

  return status0.digital_sensor;
}

double HAL_GetREVPHCompressorCurrent(HAL_REVPHHandle handle, int32_t* status) {
  auto ph = REVPHHandles->Get(handle);
  if (ph == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return 0;
  }

  PH_status1_t status1 = HAL_REV_ReadPHStatus1(ph->hcan, status);

  if (*status != 0) {
    return 0;
  }

  return PH_status1_compressor_current_decode(status1.compressor_current);
}

double HAL_GetREVPHAnalogPressure(HAL_REVPHHandle handle, int32_t channel,
                                  int32_t* status) {
  auto ph = REVPHHandles->Get(handle);
  if (ph == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return 0;
  }

  if (channel < 0 || channel > 1) {
    *status = PARAMETER_OUT_OF_RANGE;
    hal::SetLastErrorIndexOutOfRange(status, "Invalid REV Analog Index", 0, 2,
                                     channel);
    return 0;
  }

  PH_status0_t status0 = HAL_REV_ReadPHStatus0(ph->hcan, status);

  if (*status != 0) {
    return 0;
  }

  if (channel == 1) {
    return PH_status0_analog_0_decode(status0.analog_0);
  }
  return PH_status0_analog_1_decode(status0.analog_1);
}

int32_t HAL_GetREVPHSolenoids(HAL_REVPHHandle handle, int32_t* status) {
  auto ph = REVPHHandles->Get(handle);
  if (ph == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return 0;
  }

  PH_status0_t status0 = HAL_REV_ReadPHStatus0(ph->hcan, status);

  if (*status != 0) {
    return 0;
  }

  uint32_t result = status0.channel_0;
  result |= status0.channel_1 << 1;
  result |= status0.channel_2 << 2;
  result |= status0.channel_3 << 3;
  result |= status0.channel_4 << 4;
  result |= status0.channel_5 << 5;
  result |= status0.channel_6 << 6;
  result |= status0.channel_7 << 7;
  result |= status0.channel_8 << 8;
  result |= status0.channel_9 << 9;
  result |= status0.channel_10 << 10;
  result |= status0.channel_11 << 11;
  result |= status0.channel_12 << 12;
  result |= status0.channel_13 << 13;
  result |= status0.channel_14 << 14;
  result |= status0.channel_15 << 15;

  return result;
}

void HAL_SetREVPHSolenoids(HAL_REVPHHandle handle, int32_t mask, int32_t values,
                           int32_t* status) {
  auto ph = REVPHHandles->Get(handle);
  if (ph == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  std::scoped_lock lock{ph->solenoidLock};
  for (int solenoid = 0; solenoid < kNumREVPHChannels; solenoid++) {
    if (mask & (1 << solenoid)) {
      // The mask bit for the solenoid is set, so we update the solenoid state
      REV_SolenoidState desiredSolenoidState =
          values & (1 << solenoid) ? kSolenoidEnabled : kSolenoidDisabled;
      HAL_REV_UpdateDesiredPHSolenoidState(ph.get(), solenoid,
                                           desiredSolenoidState);
    }
  }
  HAL_REV_SendSolenoidsState(ph.get(), status);
}

void HAL_FireREVPHOneShot(HAL_REVPHHandle handle, int32_t index, int32_t durMs,
                          int32_t* status) {
  auto ph = REVPHHandles->Get(handle);
  if (ph == nullptr) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  if (index >= kNumREVPHChannels || index < 0) {
    *status = PARAMETER_OUT_OF_RANGE;
    hal::SetLastError(
        status,
        fmt::format("Only [0-15] are valid index values. Requested {}", index));
    return;
  }

  if (!HAL_REV_CheckPHPulseTime(durMs)) {
    *status = PARAMETER_OUT_OF_RANGE;
    hal::SetLastError(
        status,
        fmt::format("Time not within expected range [0-65534]. Requested {}",
                    durMs));
    return;
  }

  {
    std::scoped_lock lock{ph->solenoidLock};
    HAL_REV_UpdateDesiredPHSolenoidState(ph.get(), index,
                                         kSolenoidControlledViaPulse);
    HAL_REV_SendSolenoidsState(ph.get(), status);
  }

  if (*status != 0) {
    return;
  }

  PH_pulse_once_t pulse = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  pulse.pulse_length_ms = durMs;

  // Specify which solenoid should be pulsed
  // The protocol supports specifying any number of solenoids to be pulsed at
  // the same time, should that functionality be exposed to users in the future.
  switch (index) {
    case 0:
      pulse.channel_0 = true;
      break;
    case 1:
      pulse.channel_1 = true;
      break;
    case 2:
      pulse.channel_2 = true;
      break;
    case 3:
      pulse.channel_3 = true;
      break;
    case 4:
      pulse.channel_4 = true;
      break;
    case 5:
      pulse.channel_5 = true;
      break;
    case 6:
      pulse.channel_6 = true;
      break;
    case 7:
      pulse.channel_7 = true;
      break;
    case 8:
      pulse.channel_8 = true;
      break;
    case 9:
      pulse.channel_9 = true;
      break;
    case 10:
      pulse.channel_10 = true;
      break;
    case 11:
      pulse.channel_11 = true;
      break;
    case 12:
      pulse.channel_12 = true;
      break;
    case 13:
      pulse.channel_13 = true;
      break;
    case 14:
      pulse.channel_14 = true;
      break;
    case 15:
      pulse.channel_15 = true;
      break;
  }

  // Send pulse command
  uint8_t packedData[PH_PULSE_ONCE_LENGTH] = {0};
  PH_pulse_once_pack(packedData, &pulse, PH_PULSE_ONCE_LENGTH);
  HAL_WriteCANPacket(ph->hcan, packedData, PH_PULSE_ONCE_LENGTH,
                     PH_PULSE_ONCE_FRAME_API, status);
}
