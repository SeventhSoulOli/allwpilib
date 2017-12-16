/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.examples.solenoid;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * This is a sample program showing the use of the solenoid classes during
 * operator control. Three buttons from a joystick will be used to control two
 * solenoids: One button to control the position of a single solenoid and the
 * other two buttons to control a double solenoid. Single solenoids can either
 * be on or off, such that the air diverted through them goes through either one
 * channel or the other. Double solenoids have three states: Off, Forward, and
 * Reverse. Forward and Reverse divert the air through the two channels and
 * correspond to the on and off of a single solenoid, but a double solenoid can
 * also be "off", where both channels are diverted to exhaust such that there is
 * no pressure in either channel. Additionally, double solenoids take up two
 * channels on your PCM whereas single solenoids only take a single channel.
 */

public class Robot extends IterativeRobot {
	private Joystick m_stick = new Joystick(0);

	// Solenoid corresponds to a single solenoid.
	private Solenoid m_solenoid = new Solenoid(0);

	// DoubleSolenoid corresponds to a double solenoid.
	private DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(1, 2);

	private static final int kSolenoidButton = 1;
	private static final int kDoubleSolenoidForward = 2;
	private static final int kDoubleSolenoidReverse = 3;

	@Override
	public void teleopPeriodic() {
		/*
		 * The output of GetRawButton is true/false depending on whether
		 * the button is pressed; Set takes a boolean for whether
		 * to use the default (false) channel or the other (true).
		 */
		m_solenoid.set(m_stick.getRawButton(kSolenoidButton));

		/*
		 * In order to set the double solenoid, we will say that if 
		 * neither button is pressed, it is off, if just one button
		 * is pressed, set the solenoid to correspond to that button,
		 * and if both are pressed, set the solenoid to Forwards.
		 */
		if (m_stick.getRawButton(kDoubleSolenoidForward)) {
			m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
		} else if (m_stick.getRawButton(kDoubleSolenoidReverse)) {
			m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
		} else {
			m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);
		}
	}
}
