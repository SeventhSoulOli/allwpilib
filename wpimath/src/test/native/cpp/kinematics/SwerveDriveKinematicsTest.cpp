// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <wpi/numbers>

#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "gtest/gtest.h"
#include "units/angular_velocity.h"

using namespace frc;

static constexpr double kEpsilon = 0.1;

class SwerveDriveKinematicsTest : public ::testing::Test {
 protected:
  Translation2d m_fl{12_m, 12_m};
  Translation2d m_fr{12_m, -12_m};
  Translation2d m_bl{-12_m, 12_m};
  Translation2d m_br{-12_m, -12_m};

  SwerveDriveKinematics<4> m_kinematics{m_fl, m_fr, m_bl, m_br};
};

TEST_F(SwerveDriveKinematicsTest, StraightLineInverseKinematics) {
  ChassisSpeeds speeds{5.0_mps, 0.0_mps, 0.0_rad_per_s};

  auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);

  EXPECT_NEAR(fl.speed.value(), 5.0, kEpsilon);
  EXPECT_NEAR(fr.speed.value(), 5.0, kEpsilon);
  EXPECT_NEAR(bl.speed.value(), 5.0, kEpsilon);
  EXPECT_NEAR(br.speed.value(), 5.0, kEpsilon);

  EXPECT_NEAR(fl.angle.Radians().value(), 0.0, kEpsilon);
  EXPECT_NEAR(fr.angle.Radians().value(), 0.0, kEpsilon);
  EXPECT_NEAR(bl.angle.Radians().value(), 0.0, kEpsilon);
  EXPECT_NEAR(br.angle.Radians().value(), 0.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, StraightLineForwardKinematics) {
  SwerveModuleState state{5.0_mps, Rotation2d()};

  auto chassisSpeeds = m_kinematics.ToChassisSpeeds(state, state, state, state);

  EXPECT_NEAR(chassisSpeeds.vx.value(), 5.0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.vy.value(), 0.0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.omega.value(), 0.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, StraightStrafeInverseKinematics) {
  ChassisSpeeds speeds{0_mps, 5_mps, 0_rad_per_s};
  auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);

  EXPECT_NEAR(fl.speed.value(), 5.0, kEpsilon);
  EXPECT_NEAR(fr.speed.value(), 5.0, kEpsilon);
  EXPECT_NEAR(bl.speed.value(), 5.0, kEpsilon);
  EXPECT_NEAR(br.speed.value(), 5.0, kEpsilon);

  EXPECT_NEAR(fl.angle.Degrees().value(), 90.0, kEpsilon);
  EXPECT_NEAR(fr.angle.Degrees().value(), 90.0, kEpsilon);
  EXPECT_NEAR(bl.angle.Degrees().value(), 90.0, kEpsilon);
  EXPECT_NEAR(br.angle.Degrees().value(), 90.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, StraightStrafeForwardKinematics) {
  SwerveModuleState state{5_mps, Rotation2d(90_deg)};
  auto chassisSpeeds = m_kinematics.ToChassisSpeeds(state, state, state, state);

  EXPECT_NEAR(chassisSpeeds.vx.value(), 0.0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.vy.value(), 5.0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.omega.value(), 0.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, TurnInPlaceInverseKinematics) {
  ChassisSpeeds speeds{0_mps, 0_mps,
                       units::radians_per_second_t(2 * wpi::numbers::pi)};
  auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);

  EXPECT_NEAR(fl.speed.value(), 106.63, kEpsilon);
  EXPECT_NEAR(fr.speed.value(), 106.63, kEpsilon);
  EXPECT_NEAR(bl.speed.value(), 106.63, kEpsilon);
  EXPECT_NEAR(br.speed.value(), 106.63, kEpsilon);

  EXPECT_NEAR(fl.angle.Degrees().value(), 135.0, kEpsilon);
  EXPECT_NEAR(fr.angle.Degrees().value(), 45.0, kEpsilon);
  EXPECT_NEAR(bl.angle.Degrees().value(), -135.0, kEpsilon);
  EXPECT_NEAR(br.angle.Degrees().value(), -45.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, TurnInPlaceForwardKinematics) {
  SwerveModuleState fl{106.629_mps, Rotation2d(135_deg)};
  SwerveModuleState fr{106.629_mps, Rotation2d(45_deg)};
  SwerveModuleState bl{106.629_mps, Rotation2d(-135_deg)};
  SwerveModuleState br{106.629_mps, Rotation2d(-45_deg)};

  auto chassisSpeeds = m_kinematics.ToChassisSpeeds(fl, fr, bl, br);

  EXPECT_NEAR(chassisSpeeds.vx.value(), 0.0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.vy.value(), 0.0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.omega.value(), 2 * wpi::numbers::pi, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, OffCenterCORRotationInverseKinematics) {
  ChassisSpeeds speeds{0_mps, 0_mps,
                       units::radians_per_second_t(2 * wpi::numbers::pi)};
  auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds, m_fl);

  EXPECT_NEAR(fl.speed.value(), 0.0, kEpsilon);
  EXPECT_NEAR(fr.speed.value(), 150.796, kEpsilon);
  EXPECT_NEAR(bl.speed.value(), 150.796, kEpsilon);
  EXPECT_NEAR(br.speed.value(), 213.258, kEpsilon);

  EXPECT_NEAR(fl.angle.Degrees().value(), 0.0, kEpsilon);
  EXPECT_NEAR(fr.angle.Degrees().value(), 0.0, kEpsilon);
  EXPECT_NEAR(bl.angle.Degrees().value(), -90.0, kEpsilon);
  EXPECT_NEAR(br.angle.Degrees().value(), -45.0, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, OffCenterCORRotationForwardKinematics) {
  SwerveModuleState fl{0.0_mps, Rotation2d(0_deg)};
  SwerveModuleState fr{150.796_mps, Rotation2d(0_deg)};
  SwerveModuleState bl{150.796_mps, Rotation2d(-90_deg)};
  SwerveModuleState br{213.258_mps, Rotation2d(-45_deg)};

  auto chassisSpeeds = m_kinematics.ToChassisSpeeds(fl, fr, bl, br);

  EXPECT_NEAR(chassisSpeeds.vx.value(), 75.398, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.vy.value(), -75.398, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.omega.value(), 2 * wpi::numbers::pi, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest,
       OffCenterCORRotationAndTranslationInverseKinematics) {
  ChassisSpeeds speeds{0_mps, 3.0_mps, 1.5_rad_per_s};
  auto [fl, fr, bl, br] =
      m_kinematics.ToSwerveModuleStates(speeds, Translation2d(24_m, 0_m));

  EXPECT_NEAR(fl.speed.value(), 23.43, kEpsilon);
  EXPECT_NEAR(fr.speed.value(), 23.43, kEpsilon);
  EXPECT_NEAR(bl.speed.value(), 54.08, kEpsilon);
  EXPECT_NEAR(br.speed.value(), 54.08, kEpsilon);

  EXPECT_NEAR(fl.angle.Degrees().value(), -140.19, kEpsilon);
  EXPECT_NEAR(fr.angle.Degrees().value(), -39.81, kEpsilon);
  EXPECT_NEAR(bl.angle.Degrees().value(), -109.44, kEpsilon);
  EXPECT_NEAR(br.angle.Degrees().value(), -70.56, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest,
       OffCenterCORRotationAndTranslationForwardKinematics) {
  SwerveModuleState fl{23.43_mps, Rotation2d(-140.19_deg)};
  SwerveModuleState fr{23.43_mps, Rotation2d(-39.81_deg)};
  SwerveModuleState bl{54.08_mps, Rotation2d(-109.44_deg)};
  SwerveModuleState br{54.08_mps, Rotation2d(-70.56_deg)};

  auto chassisSpeeds = m_kinematics.ToChassisSpeeds(fl, fr, bl, br);

  EXPECT_NEAR(chassisSpeeds.vx.value(), 0.0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.vy.value(), -33.0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.omega.value(), 1.5, kEpsilon);
}

TEST_F(SwerveDriveKinematicsTest, Desaturate) {
  SwerveModuleState state1{5.0_mps, Rotation2d()};
  SwerveModuleState state2{6.0_mps, Rotation2d()};
  SwerveModuleState state3{4.0_mps, Rotation2d()};
  SwerveModuleState state4{7.0_mps, Rotation2d()};

  wpi::array<SwerveModuleState, 4> arr{state1, state2, state3, state4};
  SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&arr, 5.5_mps);

  double kFactor = 5.5 / 7.0;

  EXPECT_NEAR(arr[0].speed.value(), 5.0 * kFactor, kEpsilon);
  EXPECT_NEAR(arr[1].speed.value(), 6.0 * kFactor, kEpsilon);
  EXPECT_NEAR(arr[2].speed.value(), 4.0 * kFactor, kEpsilon);
  EXPECT_NEAR(arr[3].speed.value(), 7.0 * kFactor, kEpsilon);
}
