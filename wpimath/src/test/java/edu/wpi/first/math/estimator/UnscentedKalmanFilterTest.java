// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.estimator;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.NumericalJacobian;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;

class UnscentedKalmanFilterTest {
  @SuppressWarnings({"LocalVariableName", "ParameterName"})
  private static Matrix<N6, N1> getDynamics(Matrix<N6, N1> x, Matrix<N2, N1> u) {
    var motors = DCMotor.getCIM(2);

    var gHigh = 7.08;
    var rb = 0.8382 / 2.0;
    var r = 0.0746125;
    var m = 63.503;
    var J = 5.6;

    var C1 =
        -Math.pow(gHigh, 2)
            * motors.KtNMPerAmp
            / (motors.KvRadPerSecPerVolt * motors.rOhms * r * r);
    var C2 = gHigh * motors.KtNMPerAmp / (motors.rOhms * r);

    var c = x.get(2, 0);
    var s = x.get(3, 0);
    var vl = x.get(4, 0);
    var vr = x.get(5, 0);

    var Vl = u.get(0, 0);
    var Vr = u.get(1, 0);

    var k1 = 1.0 / m + rb * rb / J;
    var k2 = 1.0 / m - rb * rb / J;

    var xvel = (vl + vr) / 2;
    var w = (vr - vl) / (2.0 * rb);

    return VecBuilder.fill(
        xvel * c,
        xvel * s,
        -s * w,
        c * w,
        k1 * ((C1 * vl) + (C2 * Vl)) + k2 * ((C1 * vr) + (C2 * Vr)),
        k2 * ((C1 * vl) + (C2 * Vl)) + k1 * ((C1 * vr) + (C2 * Vr)));
  }

  @SuppressWarnings({"PMD.UnusedFormalParameter", "ParameterName"})
  private static Matrix<N4, N1> getLocalMeasurementModel(Matrix<N6, N1> x, Matrix<N2, N1> u) {
    return VecBuilder.fill(x.get(2, 0), x.get(3, 0), x.get(4, 0), x.get(5, 0));
  }

  @SuppressWarnings({"PMD.UnusedFormalParameter", "ParameterName"})
  private static Matrix<N6, N1> getGlobalMeasurementModel(Matrix<N6, N1> x, Matrix<N2, N1> u) {
    return x.copy();
  }

  @Test
  @SuppressWarnings("LocalVariableName")
  void testInit() {
    assertDoesNotThrow(
        () -> {
          UnscentedKalmanFilter<N6, N2, N4> observer =
              new UnscentedKalmanFilter<>(
                  Nat.N6(),
                  Nat.N4(),
                  UnscentedKalmanFilterTest::getDynamics,
                  UnscentedKalmanFilterTest::getLocalMeasurementModel,
                  VecBuilder.fill(0.5, 0.5, 0.7, 0.7, 1.0, 1.0),
                  VecBuilder.fill(0.001, 0.001, 0.5, 0.5),
                  0.00505);

          var u = VecBuilder.fill(12.0, 12.0);
          observer.predict(u, 0.00505);

          var localY = getLocalMeasurementModel(observer.getXhat(), u);
          observer.correct(u, localY);
        });
  }

  @SuppressWarnings("LocalVariableName")
  @Test
  void testConvergence() {
    double dtSeconds = 0.00505;
    double rbMeters = 0.8382 / 2.0; // Robot radius

    UnscentedKalmanFilter<N6, N2, N4> observer =
        new UnscentedKalmanFilter<>(
            Nat.N6(),
            Nat.N4(),
            UnscentedKalmanFilterTest::getDynamics,
            UnscentedKalmanFilterTest::getLocalMeasurementModel,
            VecBuilder.fill(0.5, 0.5, 0.7, 0.7, 1.0, 1.0),
            VecBuilder.fill(0.001, 0.001, 0.5, 0.5),
            dtSeconds);

    List<Pose2d> waypoints =
        Arrays.asList(
            new Pose2d(2.75, 22.521, new Rotation2d()),
            new Pose2d(24.73, 19.68, Rotation2d.fromDegrees(5.846)));
    var trajectory =
        TrajectoryGenerator.generateTrajectory(waypoints, new TrajectoryConfig(8.8, 0.1));

    Matrix<N6, N1> nextR;
    Matrix<N2, N1> u = new Matrix<>(Nat.N2(), Nat.N1());

    var B =
        NumericalJacobian.numericalJacobianU(
            Nat.N6(),
            Nat.N2(),
            UnscentedKalmanFilterTest::getDynamics,
            new Matrix<>(Nat.N6(), Nat.N1()),
            u);

    observer.setXhat(VecBuilder.fill(2.75, 22.521, 1.0, 0.0, 0.0, 0.0)); // TODO not hard code this

    var ref = trajectory.sample(0.0);

    Matrix<N6, N1> r =
        VecBuilder.fill(
            ref.poseMeters.getTranslation().getX(),
            ref.poseMeters.getTranslation().getY(),
            ref.poseMeters.getRotation().getCos(),
            ref.poseMeters.getRotation().getSin(),
            ref.velocityMetersPerSecond * (1 - (ref.curvatureRadPerMeter * rbMeters)),
            ref.velocityMetersPerSecond * (1 + (ref.curvatureRadPerMeter * rbMeters)));
    nextR = r.copy();

    var trueXhat = observer.getXhat();

    double totalTime = trajectory.getTotalTimeSeconds();
    for (int i = 0; i < (totalTime / dtSeconds); i++) {
      ref = trajectory.sample(dtSeconds * i);
      double vl = ref.velocityMetersPerSecond * (1 - (ref.curvatureRadPerMeter * rbMeters));
      double vr = ref.velocityMetersPerSecond * (1 + (ref.curvatureRadPerMeter * rbMeters));

      nextR.set(0, 0, ref.poseMeters.getTranslation().getX());
      nextR.set(1, 0, ref.poseMeters.getTranslation().getY());
      nextR.set(2, 0, ref.poseMeters.getRotation().getCos());
      nextR.set(3, 0, ref.poseMeters.getRotation().getSin());
      nextR.set(4, 0, vl);
      nextR.set(5, 0, vr);

      Matrix<N4, N1> localY = getLocalMeasurementModel(trueXhat, new Matrix<>(Nat.N2(), Nat.N1()));
      var noiseStdDev = VecBuilder.fill(0.001, 0.001, 0.5, 0.5);

      observer.correct(u, localY.plus(StateSpaceUtil.makeWhiteNoiseVector(noiseStdDev)));

      var rdot = nextR.minus(r).div(dtSeconds);
      u = new Matrix<>(B.solve(rdot.minus(getDynamics(r, new Matrix<>(Nat.N2(), Nat.N1())))));

      r = nextR;
      observer.predict(u, dtSeconds);
      trueXhat =
          NumericalIntegration.rk4(UnscentedKalmanFilterTest::getDynamics, trueXhat, u, dtSeconds);
    }

    var localY = getLocalMeasurementModel(trueXhat, u);
    observer.correct(u, localY);

    var globalY = getGlobalMeasurementModel(trueXhat, u);
    var R = StateSpaceUtil.makeCostMatrix(VecBuilder.fill(0.01, 0.01, 0.0001, 0.0001, 0.5, 0.5));
    observer.correct(
        Nat.N6(),
        u,
        globalY,
        UnscentedKalmanFilterTest::getGlobalMeasurementModel,
        R,
        (sigmas, weights) -> sigmas.times(Matrix.changeBoundsUnchecked(weights)),
        Matrix::minus,
        Matrix::minus,
        Matrix::plus);

    final var finalPosition = trajectory.sample(trajectory.getTotalTimeSeconds());

    assertEquals(finalPosition.poseMeters.getTranslation().getX(), observer.getXhat(0), 0.25);
    assertEquals(finalPosition.poseMeters.getTranslation().getY(), observer.getXhat(1), 0.25);
    assertEquals(finalPosition.poseMeters.getRotation().getRadians(), observer.getXhat(2), 1.0);
    assertEquals(0.0, observer.getXhat(3), 1.0);
    assertEquals(0.0, observer.getXhat(4), 1.0);
  }

  @Test
  @SuppressWarnings({"LocalVariableName", "ParameterName"})
  void testLinearUKF() {
    var dt = 0.020;
    var plant = LinearSystemId.identifyVelocitySystem(0.02, 0.006);
    var observer =
        new UnscentedKalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            (x, u) -> plant.getA().times(x).plus(plant.getB().times(u)),
            plant::calculateY,
            VecBuilder.fill(0.05),
            VecBuilder.fill(1.0),
            dt);

    var discABPair = Discretization.discretizeAB(plant.getA(), plant.getB(), dt);
    var discA = discABPair.getFirst();
    var discB = discABPair.getSecond();

    Matrix<N1, N1> ref = VecBuilder.fill(100);
    Matrix<N1, N1> u = VecBuilder.fill(0);

    for (int i = 0; i < (2.0 / dt); i++) {
      observer.predict(u, dt);

      u = discB.solve(ref.minus(discA.times(ref)));
    }

    assertEquals(ref.get(0, 0), observer.getXhat(0), 5);
  }

  @Test
  void testUnscentedTransform() {
    // From FilterPy
    var ret =
        UnscentedKalmanFilter.unscentedTransform(
            Nat.N4(),
            Nat.N4(),
            Matrix.mat(Nat.N4(), Nat.N9())
                .fill(
                    -0.9,
                    -0.822540333075852,
                    -0.8922540333075852,
                    -0.9,
                    -0.9,
                    -0.9774596669241481,
                    -0.9077459666924148,
                    -0.9,
                    -0.9,
                    1.0,
                    1.0,
                    1.077459666924148,
                    1.0,
                    1.0,
                    1.0,
                    0.9225403330758519,
                    1.0,
                    1.0,
                    -0.9,
                    -0.9,
                    -0.9,
                    -0.822540333075852,
                    -0.8922540333075852,
                    -0.9,
                    -0.9,
                    -0.9774596669241481,
                    -0.9077459666924148,
                    1.0,
                    1.0,
                    1.0,
                    1.0,
                    1.077459666924148,
                    1.0,
                    1.0,
                    1.0,
                    0.9225403330758519),
            VecBuilder.fill(
                -132.33333333,
                16.66666667,
                16.66666667,
                16.66666667,
                16.66666667,
                16.66666667,
                16.66666667,
                16.66666667,
                16.66666667),
            VecBuilder.fill(
                -129.34333333,
                16.66666667,
                16.66666667,
                16.66666667,
                16.66666667,
                16.66666667,
                16.66666667,
                16.66666667,
                16.66666667),
            (sigmas, weights) -> sigmas.times(Matrix.changeBoundsUnchecked(weights)),
            Matrix::minus);

    assertTrue(VecBuilder.fill(-0.9, 1, -0.9, 1).isEqual(ret.getFirst(), 1E-5));

    assertTrue(
        Matrix.mat(Nat.N4(), Nat.N4())
            .fill(
                2.02000002e-01,
                2.00000500e-02,
                -2.69044710e-29,
                -4.59511477e-29,
                2.00000500e-02,
                2.00001000e-01,
                -2.98781068e-29,
                -5.12759588e-29,
                -2.73372625e-29,
                -3.09882635e-29,
                2.02000002e-01,
                2.00000500e-02,
                -4.67065917e-29,
                -5.10705197e-29,
                2.00000500e-02,
                2.00001000e-01)
            .isEqual(ret.getSecond(), 1E-5));
  }
}
