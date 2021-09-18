// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "Eigen/Core"
#include "Eigen/QR"
#include "frc/StateSpaceUtil.h"
#include "frc/estimator/ExtendedKalmanFilter.h"
#include "frc/system/NumericalJacobian.h"
#include "frc/system/plant/DCMotor.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "units/moment_of_inertia.h"

namespace {

Eigen::Vector<double, 5> Dynamics(const Eigen::Vector<double, 5>& x,
                                  const Eigen::Vector<double, 2>& u) {
  auto motors = frc::DCMotor::CIM(2);

  // constexpr double Glow = 15.32;       // Low gear ratio
  constexpr double Ghigh = 7.08;       // High gear ratio
  constexpr auto rb = 0.8382_m / 2.0;  // Robot radius
  constexpr auto r = 0.0746125_m;      // Wheel radius
  constexpr auto m = 63.503_kg;        // Robot mass
  constexpr auto J = 5.6_kg_sq_m;      // Robot moment of inertia

  auto C1 = -std::pow(Ghigh, 2) * motors.Kt /
            (motors.Kv * motors.R * units::math::pow<2>(r));
  auto C2 = Ghigh * motors.Kt / (motors.R * r);
  auto k1 = (1 / m + units::math::pow<2>(rb) / J);
  auto k2 = (1 / m - units::math::pow<2>(rb) / J);

  units::meters_per_second_t vl{x(3)};
  units::meters_per_second_t vr{x(4)};
  units::volt_t Vl{u(0)};
  units::volt_t Vr{u(1)};

  auto v = 0.5 * (vl + vr);
  return Eigen::Vector<double, 5>{
      v.to<double>() * std::cos(x(2)), v.to<double>() * std::sin(x(2)),
      ((vr - vl) / (2.0 * rb)).to<double>(),
      k1.to<double>() * ((C1 * vl).to<double>() + (C2 * Vl).to<double>()) +
          k2.to<double>() * ((C1 * vr).to<double>() + (C2 * Vr).to<double>()),
      k2.to<double>() * ((C1 * vl).to<double>() + (C2 * Vl).to<double>()) +
          k1.to<double>() * ((C1 * vr).to<double>() + (C2 * Vr).to<double>())};
}

Eigen::Vector<double, 3> LocalMeasurementModel(
    const Eigen::Vector<double, 5>& x, const Eigen::Vector<double, 2>& u) {
  static_cast<void>(u);
  return Eigen::Vector<double, 3>{x(2), x(3), x(4)};
}

Eigen::Vector<double, 5> GlobalMeasurementModel(
    const Eigen::Vector<double, 5>& x, const Eigen::Vector<double, 2>& u) {
  static_cast<void>(u);
  return Eigen::Vector<double, 5>{x(0), x(1), x(2), x(3), x(4)};
}
}  // namespace

TEST(ExtendedKalmanFilterTest, Init) {
  constexpr auto dt = 0.00505_s;

  frc::ExtendedKalmanFilter<5, 2, 3> observer{Dynamics,
                                              LocalMeasurementModel,
                                              {0.5, 0.5, 10.0, 1.0, 1.0},
                                              {0.0001, 0.01, 0.01},
                                              dt};
  Eigen::Vector<double, 2> u{12.0, 12.0};
  observer.Predict(u, dt);

  auto localY = LocalMeasurementModel(observer.Xhat(), u);
  observer.Correct(u, localY);

  auto globalY = GlobalMeasurementModel(observer.Xhat(), u);
  auto R = frc::MakeCovMatrix(0.01, 0.01, 0.0001, 0.01, 0.01);
  observer.Correct<5>(u, globalY, GlobalMeasurementModel, R);
}

TEST(ExtendedKalmanFilterTest, Convergence) {
  constexpr auto dt = 0.00505_s;
  constexpr auto rb = 0.8382_m / 2.0;  // Robot radius

  frc::ExtendedKalmanFilter<5, 2, 3> observer{Dynamics,
                                              LocalMeasurementModel,
                                              {0.5, 0.5, 10.0, 1.0, 1.0},
                                              {0.0001, 0.5, 0.5},
                                              dt};

  auto waypoints =
      std::vector<frc::Pose2d>{frc::Pose2d{2.75_m, 22.521_m, 0_rad},
                               frc::Pose2d{24.73_m, 19.68_m, 5.846_rad}};
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      waypoints, {8.8_mps, 0.1_mps_sq});

  Eigen::Vector<double, 5> r = Eigen::Vector<double, 5>::Zero();
  Eigen::Vector<double, 2> u = Eigen::Vector<double, 2>::Zero();

  auto B = frc::NumericalJacobianU<5, 5, 2>(Dynamics,
                                            Eigen::Vector<double, 5>::Zero(),
                                            Eigen::Vector<double, 2>::Zero());

  observer.SetXhat(Eigen::Vector<double, 5>{
      trajectory.InitialPose().Translation().X().to<double>(),
      trajectory.InitialPose().Translation().Y().to<double>(),
      trajectory.InitialPose().Rotation().Radians().to<double>(), 0.0, 0.0});

  auto totalTime = trajectory.TotalTime();
  for (size_t i = 0; i < (totalTime / dt).to<double>(); ++i) {
    auto ref = trajectory.Sample(dt * i);
    units::meters_per_second_t vl =
        ref.velocity * (1 - (ref.curvature * rb).to<double>());
    units::meters_per_second_t vr =
        ref.velocity * (1 + (ref.curvature * rb).to<double>());

    Eigen::Vector<double, 5> nextR{ref.pose.Translation().X().to<double>(),
                                   ref.pose.Translation().Y().to<double>(),
                                   ref.pose.Rotation().Radians().to<double>(),
                                   vl.to<double>(), vr.to<double>()};

    auto localY =
        LocalMeasurementModel(nextR, Eigen::Vector<double, 2>::Zero());
    observer.Correct(u, localY + frc::MakeWhiteNoiseVector(0.0001, 0.5, 0.5));

    Eigen::Vector<double, 5> rdot = (nextR - r) / dt.to<double>();
    u = B.householderQr().solve(rdot -
                                Dynamics(r, Eigen::Vector<double, 2>::Zero()));

    observer.Predict(u, dt);

    r = nextR;
  }

  auto localY = LocalMeasurementModel(observer.Xhat(), u);
  observer.Correct(u, localY);

  auto globalY = GlobalMeasurementModel(observer.Xhat(), u);
  auto R = frc::MakeCovMatrix(0.01, 0.01, 0.0001, 0.5, 0.5);
  observer.Correct<5>(u, globalY, GlobalMeasurementModel, R);

  auto finalPosition = trajectory.Sample(trajectory.TotalTime());
  ASSERT_NEAR(finalPosition.pose.Translation().X().template to<double>(),
              observer.Xhat(0), 1.0);
  ASSERT_NEAR(finalPosition.pose.Translation().Y().template to<double>(),
              observer.Xhat(1), 1.0);
  ASSERT_NEAR(finalPosition.pose.Rotation().Radians().template to<double>(),
              observer.Xhat(2), 1.0);
  ASSERT_NEAR(0.0, observer.Xhat(3), 1.0);
  ASSERT_NEAR(0.0, observer.Xhat(4), 1.0);
}
