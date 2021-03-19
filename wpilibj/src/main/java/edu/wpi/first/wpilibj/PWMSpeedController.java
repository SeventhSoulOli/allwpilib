// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/** Common base class for all PWM Speed Controllers. */
public abstract class PWMSpeedController extends MotorSafety
    implements SpeedController, Sendable, AutoCloseable {
  private boolean m_isInverted;
  protected PWM m_pwm;

  /**
   * Constructor.
   *
   * @param name Name to use for SendableRegistry
   * @param channel The PWM channel that the controller is attached to. 0-9 are on-board, 10-19 are
   *     on the MXP port
   */
  protected PWMSpeedController(final String name, final int channel) {
    m_pwm = new PWM(channel, false);
    SendableRegistry.addLW(this, name, channel);
  }

  /** Free the resource associated with the PWM channel and set the value to 0. */
  @Override
  public void close() {
    SendableRegistry.remove(this);
    m_pwm.close();
  }

  /**
   * Set the PWM value.
   *
   * <p>The PWM value is set using a range of -1.0 to 1.0, appropriately scaling the value for the
   * FPGA.
   *
   * @param speed The speed value between -1.0 and 1.0 to set.
   */
  @Override
  public void set(double speed) {
    m_pwm.setSpeed(m_isInverted ? -speed : speed);
    feed();
  }

  /**
   * Get the recently set value of the PWM. This value is affected by the inversion property. If you
   * want the value that is sent directly to the SpeedController, use {@link
   * edu.wpi.first.wpilibj.PWM#getSpeed()} instead.
   *
   * @return The most recently set value for the PWM between -1.0 and 1.0.
   */
  @Override
  public double get() {
    return m_pwm.getSpeed() * (m_isInverted ? -1.0 : 1.0);
  }

  @Override
  public void setInverted(boolean isInverted) {
    m_isInverted = isInverted;
  }

  @Override
  public boolean getInverted() {
    return m_isInverted;
  }

  @Override
  public void disable() {
    m_pwm.setDisabled();
  }

  @Override
  public void stopMotor() {
    disable();
  }

  @Override
  public String getDescription() {
    return "PWM " + getChannel();
  }

  /**
   * Gets the PWM channel number.
   *
   * @return The channel number.
   */
  public int getChannel() {
    return m_pwm.getChannel();
  }

  /**
   * Write out the PID value as seen in the PIDOutput base object.
   *
   * @param output Write out the PWM value as was found in the PIDController
   */
  @Override
  public void pidWrite(double output) {
    set(output);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Speed Controller");
    builder.setActuator(true);
    builder.setSafeState(this::disable);
    builder.addDoubleProperty("Value", this::get, this::set);
  }
}
