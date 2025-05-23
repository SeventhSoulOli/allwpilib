// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/SymbolExports.h>
#include <wpi/protobuf/Protobuf.h>

#include "frc/geometry/Twist3d.h"
#include "wpimath/protobuf/geometry3d.npb.h"

template <>
struct WPILIB_DLLEXPORT wpi::Protobuf<frc::Twist3d> {
  using MessageStruct = wpi_proto_ProtobufTwist3d;
  using InputStream = wpi::ProtoInputStream<frc::Twist3d>;
  using OutputStream = wpi::ProtoOutputStream<frc::Twist3d>;
  static std::optional<frc::Twist3d> Unpack(InputStream& stream);
  static bool Pack(OutputStream& stream, const frc::Twist3d& value);
};
