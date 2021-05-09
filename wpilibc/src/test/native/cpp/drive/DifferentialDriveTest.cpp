// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "MockMotorController.h"
#include "frc/drive/DifferentialDrive.h"
#include "gtest/gtest.h"

TEST(DifferentialDriveTest, ArcadeDrive) {
  frc::MockMotorController left;
  frc::MockMotorController right;
  frc::DifferentialDrive drive{left, right};
  drive.SetDeadband(0.0);

  // Forward
  drive.ArcadeDrive(1.0, 0.0, false);
  EXPECT_DOUBLE_EQ(1.0, left.Get());
  EXPECT_DOUBLE_EQ(1.0, right.Get());

  // Forward left turn
  drive.ArcadeDrive(0.5, -0.5, false);
  EXPECT_DOUBLE_EQ(0.0, left.Get());
  EXPECT_DOUBLE_EQ(0.5, right.Get());

  // Forward right turn
  drive.ArcadeDrive(0.5, 0.5, false);
  EXPECT_DOUBLE_EQ(0.5, left.Get());
  EXPECT_DOUBLE_EQ(0.0, right.Get());

  // Backward
  drive.ArcadeDrive(-1.0, 0.0, false);
  EXPECT_DOUBLE_EQ(-1.0, left.Get());
  EXPECT_DOUBLE_EQ(-1.0, right.Get());

  // Backward left turn
  drive.ArcadeDrive(-0.5, -0.5, false);
  EXPECT_DOUBLE_EQ(-0.5, left.Get());
  EXPECT_DOUBLE_EQ(0.0, right.Get());

  // Backward right turn
  drive.ArcadeDrive(-0.5, 0.5, false);
  EXPECT_DOUBLE_EQ(0.0, left.Get());
  EXPECT_DOUBLE_EQ(-0.5, right.Get());
}

TEST(DifferentialDriveTest, ArcadeDriveSquared) {
  frc::MockMotorController left;
  frc::MockMotorController right;
  frc::DifferentialDrive drive{left, right};
  drive.SetDeadband(0.0);

  // Forward
  drive.ArcadeDrive(1.0, 0.0, true);
  EXPECT_DOUBLE_EQ(1.0, left.Get());
  EXPECT_DOUBLE_EQ(1.0, right.Get());

  // Forward left turn
  drive.ArcadeDrive(0.5, -0.5, true);
  EXPECT_DOUBLE_EQ(0.0, left.Get());
  EXPECT_DOUBLE_EQ(0.25, right.Get());

  // Forward right turn
  drive.ArcadeDrive(0.5, 0.5, true);
  EXPECT_DOUBLE_EQ(0.25, left.Get());
  EXPECT_DOUBLE_EQ(0.0, right.Get());

  // Backward
  drive.ArcadeDrive(-1.0, 0.0, true);
  EXPECT_DOUBLE_EQ(-1.0, left.Get());
  EXPECT_DOUBLE_EQ(-1.0, right.Get());

  // Backward left turn
  drive.ArcadeDrive(-0.5, -0.5, true);
  EXPECT_DOUBLE_EQ(-0.25, left.Get());
  EXPECT_DOUBLE_EQ(0.0, right.Get());

  // Backward right turn
  drive.ArcadeDrive(-0.5, 0.5, true);
  EXPECT_DOUBLE_EQ(0.0, left.Get());
  EXPECT_DOUBLE_EQ(-0.25, right.Get());
}

TEST(DifferentialDriveTest, CurvatureDrive) {
  frc::MockMotorController left;
  frc::MockMotorController right;
  frc::DifferentialDrive drive{left, right};
  drive.SetDeadband(0.0);

  // Forward
  drive.CurvatureDrive(1.0, 0.0, false);
  EXPECT_DOUBLE_EQ(1.0, left.Get());
  EXPECT_DOUBLE_EQ(1.0, right.Get());

  // Forward left turn
  drive.CurvatureDrive(0.5, -0.5, false);
  EXPECT_DOUBLE_EQ(0.25, left.Get());
  EXPECT_DOUBLE_EQ(0.75, right.Get());

  // Forward right turn
  drive.CurvatureDrive(0.5, 0.5, false);
  EXPECT_DOUBLE_EQ(0.75, left.Get());
  EXPECT_DOUBLE_EQ(0.25, right.Get());

  // Backward
  drive.CurvatureDrive(-1.0, 0.0, false);
  EXPECT_DOUBLE_EQ(-1.0, left.Get());
  EXPECT_DOUBLE_EQ(-1.0, right.Get());

  // Backward left turn
  drive.CurvatureDrive(-0.5, -0.5, false);
  EXPECT_DOUBLE_EQ(-0.75, left.Get());
  EXPECT_DOUBLE_EQ(-0.25, right.Get());

  // Backward right turn
  drive.CurvatureDrive(-0.5, 0.5, false);
  EXPECT_DOUBLE_EQ(-0.25, left.Get());
  EXPECT_DOUBLE_EQ(-0.75, right.Get());
}

TEST(DifferentialDriveTest, CurvatureDriveTurnInPlace) {
  frc::MockMotorController left;
  frc::MockMotorController right;
  frc::DifferentialDrive drive{left, right};
  drive.SetDeadband(0.0);

  // Forward
  drive.CurvatureDrive(1.0, 0.0, true);
  EXPECT_DOUBLE_EQ(1.0, left.Get());
  EXPECT_DOUBLE_EQ(1.0, right.Get());

  // Forward left turn
  drive.CurvatureDrive(0.5, -0.5, true);
  EXPECT_DOUBLE_EQ(0.0, left.Get());
  EXPECT_DOUBLE_EQ(1.0, right.Get());

  // Forward right turn
  drive.CurvatureDrive(0.5, 0.5, true);
  EXPECT_DOUBLE_EQ(1.0, left.Get());
  EXPECT_DOUBLE_EQ(0.0, right.Get());

  // Backward
  drive.CurvatureDrive(-1.0, 0.0, true);
  EXPECT_DOUBLE_EQ(-1.0, left.Get());
  EXPECT_DOUBLE_EQ(-1.0, right.Get());

  // Backward left turn
  drive.CurvatureDrive(-0.5, -0.5, true);
  EXPECT_DOUBLE_EQ(-1.0, left.Get());
  EXPECT_DOUBLE_EQ(0.0, right.Get());

  // Backward right turn
  drive.CurvatureDrive(-0.5, 0.5, true);
  EXPECT_DOUBLE_EQ(0.0, left.Get());
  EXPECT_DOUBLE_EQ(-1.0, right.Get());
}

TEST(DifferentialDriveTest, TankDrive) {
  frc::MockMotorController left;
  frc::MockMotorController right;
  frc::DifferentialDrive drive{left, right};
  drive.SetDeadband(0.0);

  // Forward
  drive.TankDrive(1.0, 1.0, false);
  EXPECT_DOUBLE_EQ(1.0, left.Get());
  EXPECT_DOUBLE_EQ(1.0, right.Get());

  // Forward left turn
  drive.TankDrive(0.5, 1.0, false);
  EXPECT_DOUBLE_EQ(0.5, left.Get());
  EXPECT_DOUBLE_EQ(1.0, right.Get());

  // Forward right turn
  drive.TankDrive(1.0, 0.5, false);
  EXPECT_DOUBLE_EQ(1.0, left.Get());
  EXPECT_DOUBLE_EQ(0.5, right.Get());

  // Backward
  drive.TankDrive(-1.0, -1.0, false);
  EXPECT_DOUBLE_EQ(-1.0, left.Get());
  EXPECT_DOUBLE_EQ(-1.0, right.Get());

  // Backward left turn
  drive.TankDrive(-0.5, -1.0, false);
  EXPECT_DOUBLE_EQ(-0.5, left.Get());
  EXPECT_DOUBLE_EQ(-1.0, right.Get());

  // Backward right turn
  drive.TankDrive(-0.5, 1.0, false);
  EXPECT_DOUBLE_EQ(-0.5, left.Get());
  EXPECT_DOUBLE_EQ(1.0, right.Get());
}

TEST(DifferentialDriveTest, TankDriveSquared) {
  frc::MockMotorController left;
  frc::MockMotorController right;
  frc::DifferentialDrive drive{left, right};
  drive.SetDeadband(0.0);

  // Forward
  drive.TankDrive(1.0, 1.0, true);
  EXPECT_DOUBLE_EQ(1.0, left.Get());
  EXPECT_DOUBLE_EQ(1.0, right.Get());

  // Forward left turn
  drive.TankDrive(0.5, 1.0, true);
  EXPECT_DOUBLE_EQ(0.25, left.Get());
  EXPECT_DOUBLE_EQ(1.0, right.Get());

  // Forward right turn
  drive.TankDrive(1.0, 0.5, true);
  EXPECT_DOUBLE_EQ(1.0, left.Get());
  EXPECT_DOUBLE_EQ(0.25, right.Get());

  // Backward
  drive.TankDrive(-1.0, -1.0, true);
  EXPECT_DOUBLE_EQ(-1.0, left.Get());
  EXPECT_DOUBLE_EQ(-1.0, right.Get());

  // Backward left turn
  drive.TankDrive(-0.5, -1.0, true);
  EXPECT_DOUBLE_EQ(-0.25, left.Get());
  EXPECT_DOUBLE_EQ(-1.0, right.Get());

  // Backward right turn
  drive.TankDrive(-1.0, -0.5, true);
  EXPECT_DOUBLE_EQ(-1.0, left.Get());
  EXPECT_DOUBLE_EQ(-0.25, right.Get());
}
