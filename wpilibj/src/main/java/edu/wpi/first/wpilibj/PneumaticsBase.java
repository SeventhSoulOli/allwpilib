// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj;

public interface PneumaticsBase extends AutoCloseable {

  /**
   * Sets solenoids on a pneumatics module.
   *
   * @param mask mask
   * @param values values
   */
  void setSolenoids(int mask, int values);

  /**
   * Gets solenoid values.
   *
   * @return values
   */
  int getSolenoids();

  /**
   * Get module number for this module.
   *
   * @return module number
   */
  int getModuleNumber();

  /**
   * Get the disabled solenoids.
   *
   * @return disabled list
   */
  int getSolenoidDisabledList();

  /**
   * Fire a single solenoid shot.
   *
   * @param index solenoid index
   */
  void fireOneShot(int index);

  /**
   * Set the duration for a single solenoid shot.
   *
   * @param index solenoid index
   * @param durMs shot duration
   */
  void setOneShotDuration(int index, int durMs);

  boolean checkSolenoidChannel(int channel);
}
