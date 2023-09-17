// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "glass/hardware/AnalogGyro.h"

#include <fmt/format.h>

#include "glass/DataSource.h"
#include "glass/other/DeviceTree.h"

using namespace glass;

void glass::DisplayAnalogGyroDevice(AnalogGyroModel* model, int index) {
  char name[32];
  const auto result =
      fmt::format_to_n(name, sizeof(name) - 1, "AnalogGyro[{}]", index);
  *result.out = '\0';

  if (BeginDevice(name)) {
    // angle
    if (auto angleData = model->GetAngleData()) {
      double value = angleData->GetValue();
      if (DeviceDouble("Angle", false, &value, angleData)) {
        model->SetAngle(value);
      }
    }

    // rate
    if (auto rateData = model->GetRateData()) {
      double value = rateData->GetValue();
      if (DeviceDouble("Rate", false, &value, rateData)) {
        model->SetRate(value);
      }
    }
    EndDevice();
  }
}

void glass::DisplayAnalogGyrosDevice(AnalogGyrosModel* model) {
  model->ForEachAnalogGyro(
      [&](AnalogGyroModel& gyro, int i) { DisplayAnalogGyroDevice(&gyro, i); });
}
