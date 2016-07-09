/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>

extern "C" {
float HAL_GetVinVoltage(int32_t* status);
float HAL_GetVinCurrent(int32_t* status);
float HAL_GetUserVoltage6V(int32_t* status);
float HAL_GetUserCurrent6V(int32_t* status);
bool HAL_GetUserActive6V(int32_t* status);
int HAL_GetUserCurrentFaults6V(int32_t* status);
float HAL_GetUserVoltage5V(int32_t* status);
float HAL_GetUserCurrent5V(int32_t* status);
bool HAL_GetUserActive5V(int32_t* status);
int HAL_GetUserCurrentFaults5V(int32_t* status);
float HAL_GetUserVoltage3V3(int32_t* status);
float HAL_GetUserCurrent3V3(int32_t* status);
bool HAL_GetUserActive3V3(int32_t* status);
int HAL_GetUserCurrentFaults3V3(int32_t* status);
}
