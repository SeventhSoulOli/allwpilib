// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/smartdashboard/MechanismRoot2d.h"

#include "frc/util/Color8Bit.h"

using namespace frc;

MechanismRoot2d::MechanismRoot2d(std::string_view name, double x, double y,
                                 const private_init&)
    : MechanismObject2d(name), m_x{x}, m_y{y} {}

void MechanismRoot2d::SetPosition(double x, double y) {
  std::scoped_lock lock(m_mutex);
  m_x = x;
  m_y = y;
  Flush();
}

void MechanismRoot2d::UpdateEntries(std::shared_ptr<nt::NetworkTable> table) {
  m_xEntry = table->GetEntry("x");
  m_yEntry = table->GetEntry("y");
  Flush();
}

inline void MechanismRoot2d::Flush() {
  if (m_xEntry) {
    m_xEntry.SetDouble(m_x);
  }
  if (m_yEntry) {
    m_yEntry.SetDouble(m_y);
  }
}
