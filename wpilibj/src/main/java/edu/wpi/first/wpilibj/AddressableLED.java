/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj;

import edu.wpi.first.hal.AddressableLEDJNI;

/**
 * A class for driving addressable LEDs, such as WS2812s and NeoPixels.
 */
public class AddressableLED implements AutoCloseable {
  private final PWM m_pwmOutput;
  private int m_handle;
  private final boolean m_ownsPwm;

  /**
   * Constructs a new driver from a PWM output.
   *
   * @param output the pwm output to use
   */
  public AddressableLED(PWM output) {
    m_pwmOutput = output;
    m_ownsPwm = false;
    init();
  }

  /**
   * Constructs a new driver for a specific port.
   *
   * @param port the output port to use (Must be a PWM port)
   */
  public AddressableLED(int port) {
    m_pwmOutput = new PWM(port);
    m_ownsPwm = true;
    init();
  }

  private void init() {
    m_handle = AddressableLEDJNI.initialize(m_pwmOutput.m_handle);
  }

  @Override
  public void close() {
    if (m_handle != 0) {
      AddressableLEDJNI.free(m_handle);
    }
    if (m_ownsPwm) {
      m_pwmOutput.close();
    }
  }

  /**
   * Sets the length of the LED strip.
   *
   * <p>Calling this is an expensive call, so its best to call it once, then just update data.
   *
   * @param length the strip length
   */
  public void setLength(int length) {
    AddressableLEDJNI.setLength(m_handle, length);
  }

  /**
   * Sets the led output data.
   *
   * <p>If the output is enabled, this will start writing the next data cycle.
   * It is safe to call, even while output is enabled.
   *
   * @param buffer the buffer to write
   */
  public void setData(AddressableLEDBuffer buffer) {
    AddressableLEDJNI.setData(m_handle, buffer.m_buffer);
  }

  /**
   * Sets the bit timing.
   *
   * <p>By default, the driver is set up to drive WS2812s, so nothing needs to be set for those.
   *
   * @param lowTime0NanoSeconds low time for 0 bit
   * @param highTime0NanoSeconds high time for 0 bit
   * @param lowTime1NanoSeconds low time for 1 bit
   * @param highTime1NanoSeconds high time for 1 bit
   */
  public void setBitTiming(int lowTime0NanoSeconds, int highTime0NanoSeconds,
      int lowTime1NanoSeconds, int highTime1NanoSeconds) {
    AddressableLEDJNI.setBitTiming(m_handle, lowTime0NanoSeconds,
        highTime0NanoSeconds, lowTime1NanoSeconds,
        highTime1NanoSeconds);
  }

  /**
   * Sets the sync time.
   *
   * <p>The sync time is the time to hold output so LEDs enable. Default set for WS2812.
   *
   * @param syncTimeMicroSeconds the sync time
   */
  public void setSyncTime(int syncTimeMicroSeconds) {
    AddressableLEDJNI.setSyncTime(m_handle, syncTimeMicroSeconds);
  }

  /**
   * Starts the output.
   *
   * <p>The output writes continously.
   */
  public void start() {
    AddressableLEDJNI.start(m_handle);
  }

  /**
   * Stops the output.
   */
  public void stop() {
    AddressableLEDJNI.stop(m_handle);
  }
}
