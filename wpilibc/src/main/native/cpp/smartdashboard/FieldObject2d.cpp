// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/smartdashboard/FieldObject2d.h"

#include <vector>

#include <wpi/Endian.h>
#include <wpi/MathExtras.h>

#include "frc/trajectory/Trajectory.h"

using namespace frc;

FieldObject2d::FieldObject2d(FieldObject2d&& rhs) {
  std::swap(m_name, rhs.m_name);
  std::swap(m_entry, rhs.m_entry);
  std::swap(m_poses, rhs.m_poses);
}

FieldObject2d& FieldObject2d::operator=(FieldObject2d&& rhs) {
  std::swap(m_name, rhs.m_name);
  std::swap(m_entry, rhs.m_entry);
  std::swap(m_poses, rhs.m_poses);

  return *this;
}

void FieldObject2d::SetPose(const Pose2d& pose) {
  SetPoses({pose});
}

void FieldObject2d::SetPose(units::meter_t x, units::meter_t y,
                            Rotation2d rotation) {
  SetPoses({{x, y, rotation}});
}

Pose2d FieldObject2d::GetPose() const {
  std::scoped_lock lock(m_mutex);
  UpdateFromEntry();
  if (m_poses.empty()) {
    return {};
  }
  return m_poses[0];
}

void FieldObject2d::SetPoses(wpi::span<const Pose2d> poses) {
  std::scoped_lock lock(m_mutex);
  m_poses.assign(poses.begin(), poses.end());
  UpdateEntry();
}

void FieldObject2d::SetPoses(std::initializer_list<Pose2d> poses) {
  SetPoses({poses.begin(), poses.end()});
}

void FieldObject2d::SetTrajectory(const Trajectory& trajectory) {
  std::scoped_lock lock(m_mutex);
  m_poses.clear();
  m_poses.reserve(trajectory.States().size());
  for (auto&& state : trajectory.States()) {
    m_poses.push_back(state.pose);
  }
  UpdateEntry();
}

std::vector<Pose2d> FieldObject2d::GetPoses() const {
  std::scoped_lock lock(m_mutex);
  UpdateFromEntry();
  return std::vector<Pose2d>(m_poses.begin(), m_poses.end());
}

wpi::span<const Pose2d> FieldObject2d::GetPoses(
    wpi::SmallVectorImpl<Pose2d>& out) const {
  std::scoped_lock lock(m_mutex);
  UpdateFromEntry();
  out.assign(m_poses.begin(), m_poses.end());
  return out;
}

void FieldObject2d::UpdateEntry(bool setDefault) {
  if (!m_entry) {
    return;
  }
  if (m_poses.size() < (255 / 3)) {
    wpi::SmallVector<double, 9> arr;
    for (auto&& pose : m_poses) {
      auto& translation = pose.Translation();
      arr.push_back(translation.X().value());
      arr.push_back(translation.Y().value());
      arr.push_back(pose.Rotation().Degrees().value());
    }
    if (setDefault) {
      m_entry.SetDefaultDoubleArray(arr);
    } else {
      m_entry.ForceSetDoubleArray(arr);
    }
  } else {
    // send as raw array of doubles if too big for NT array
    std::vector<char> arr;
    arr.resize(m_poses.size() * 3 * 8);
    char* p = arr.data();
    for (auto&& pose : m_poses) {
      auto& translation = pose.Translation();
      wpi::support::endian::write64be(
          p, wpi::DoubleToBits(translation.X().value()));
      p += 8;
      wpi::support::endian::write64be(
          p, wpi::DoubleToBits(translation.Y().value()));
      p += 8;
      wpi::support::endian::write64be(
          p, wpi::DoubleToBits(pose.Rotation().Degrees().value()));
      p += 8;
    }
    if (setDefault) {
      m_entry.SetDefaultRaw({arr.data(), arr.size()});
    } else {
      m_entry.ForceSetRaw({arr.data(), arr.size()});
    }
  }
}

void FieldObject2d::UpdateFromEntry() const {
  if (!m_entry) {
    return;
  }
  auto val = m_entry.GetValue();
  if (!val) {
    return;
  }

  if (val->IsDoubleArray()) {
    auto arr = val->GetDoubleArray();
    auto size = arr.size();
    if ((size % 3) != 0) {
      return;
    }
    m_poses.resize(size / 3);
    for (size_t i = 0; i < size / 3; ++i) {
      m_poses[i] =
          Pose2d{units::meter_t{arr[i * 3 + 0]}, units::meter_t{arr[i * 3 + 1]},
                 units::degree_t{arr[i * 3 + 2]}};
    }
  } else if (val->IsRaw()) {
    // treat it simply as an array of doubles
    std::string_view data = val->GetRaw();

    // must be triples of doubles
    auto size = data.size();
    if ((size % (3 * 8)) != 0) {
      return;
    }
    m_poses.resize(size / (3 * 8));
    const char* p = data.data();
    for (size_t i = 0; i < size / (3 * 8); ++i) {
      double x = wpi::BitsToDouble(
          wpi::support::endian::readNext<uint64_t, wpi::support::big,
                                         wpi::support::unaligned>(p));
      double y = wpi::BitsToDouble(
          wpi::support::endian::readNext<uint64_t, wpi::support::big,
                                         wpi::support::unaligned>(p));
      double rot = wpi::BitsToDouble(
          wpi::support::endian::readNext<uint64_t, wpi::support::big,
                                         wpi::support::unaligned>(p));
      m_poses[i] =
          Pose2d{units::meter_t{x}, units::meter_t{y}, units::degree_t{rot}};
    }
  }
}
