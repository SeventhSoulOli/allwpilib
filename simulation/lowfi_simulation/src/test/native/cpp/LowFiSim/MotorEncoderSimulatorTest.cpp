/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Encoder.h"
#include "LowFiSim/MotorEncoderConnector.h"
#include "LowFiSim/MotorModel/SimpleMotorModel.h"
#include "LowFiSim/WpiSimulators/WpiEncoderSim.h"
#include "LowFiSim/WpiSimulators/WpiMotorSim.h"
#include "Talon.h"
#include "gtest/gtest.h"

TEST(MotorEncoderConnectorTest, TestWithoutDistancePerPulseFullSpeed) {
  frc::Talon talon{3};
  frc::Encoder encoder{3, 1};

  frc::sim::lowfi::SimpleMotorModel motorModelSim(6000);
  frc::sim::lowfi::WpiMotorSim motorSim(3, motorModelSim);
  frc::sim::lowfi::WpiEncoderSim encoderSim(0);
  frc::sim::lowfi::MotorEncoderConnector connector(motorSim, encoderSim);

  talon.Set(-1);
  motorSim.Update(1);
  connector.Update();

  // Position
  EXPECT_EQ(-6000, encoder.Get());
  EXPECT_DOUBLE_EQ(-6000, encoder.GetDistance());

  // Velocity
  EXPECT_DOUBLE_EQ(-1.0 / 6000, encoder.GetPeriod());
  EXPECT_DOUBLE_EQ(-6000, encoder.GetRate());
}

TEST(MotorEncoderConnectorTest, TestWithoutDistancePerPulseRealisitcUpdate) {
  frc::Talon talon{3};
  frc::Encoder encoder{3, 1};

  frc::sim::lowfi::SimpleMotorModel motorModelSim(6000);
  frc::sim::lowfi::WpiMotorSim motorSim(3, motorModelSim);
  frc::sim::lowfi::WpiEncoderSim encoderSim(0);
  frc::sim::lowfi::MotorEncoderConnector connector(motorSim, encoderSim);

  talon.Set(0.5);
  motorSim.Update(.02);
  connector.Update();

  // Position
  EXPECT_EQ(60, encoder.Get());
  EXPECT_DOUBLE_EQ(60, encoder.GetDistance());

  // Velocity
  EXPECT_DOUBLE_EQ(1.0 / 3000, encoder.GetPeriod());
  EXPECT_DOUBLE_EQ(3000, encoder.GetRate());
}

TEST(MotorEncoderConnectorTest, TestWithDistancePerPulseFullSpeed) {
  frc::Talon talon{3};
  frc::Encoder encoder{3, 1};
  encoder.SetDistancePerPulse(.001);

  frc::sim::lowfi::SimpleMotorModel motorModelSim(6000);
  frc::sim::lowfi::WpiMotorSim motorSim(3, motorModelSim);
  frc::sim::lowfi::WpiEncoderSim encoderSim(0);
  frc::sim::lowfi::MotorEncoderConnector connector(motorSim, encoderSim);

  talon.Set(-1);

  motorSim.Update(1);
  connector.Update();

  // Position
  EXPECT_EQ(-6000000, encoder.Get());
  EXPECT_DOUBLE_EQ(-6000, encoder.GetDistance());

  // Velocity
  EXPECT_EQ(-1.0 / 6000, encoder.GetPeriod());
  EXPECT_DOUBLE_EQ(-6, encoder.GetRate());
}

TEST(MotorEncoderConnectorTest, TestWithDistancePerPulseRealistic) {
  frc::Talon talon{3};
  frc::Encoder encoder{3, 1};
  encoder.SetDistancePerPulse(.001);

  frc::sim::lowfi::SimpleMotorModel motorModelSim(6000);
  frc::sim::lowfi::WpiMotorSim motorSim(3, motorModelSim);
  frc::sim::lowfi::WpiEncoderSim encoderSim(0);
  frc::sim::lowfi::MotorEncoderConnector connector(motorSim, encoderSim);

  talon.Set(0.5);

  motorSim.Update(.02);
  connector.Update();

  // Position
  EXPECT_EQ(60000, encoder.Get());
  EXPECT_DOUBLE_EQ(60, encoder.GetDistance());

  // Velocity
  EXPECT_EQ(1.0 / 3000, encoder.GetPeriod());
  EXPECT_DOUBLE_EQ(3, encoder.GetRate());
}
