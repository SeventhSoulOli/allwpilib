// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj2.command.button;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;

/**
 * This class provides an easy way to link commands to conditions.
 *
 * <p>It is very easy to link a button to a command. For instance, you could link the trigger button
 * of a joystick to a "score" command.
 *
 * <p>Triggers can easily be composed for advanced functionality using the {@link
 * #and(BooleanSupplier)}, {@link #or(BooleanSupplier)}, {@link #negate()} operators.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class Trigger implements BooleanSupplier {
  private final BooleanEvent m_event;

  /**
   * Creates a new trigger based on the given condition.
   *
   * @param loop The loop instance that polls this trigger.
   * @param condition the condition represented by this trigger
   */
  public Trigger(EventLoop loop, BooleanSupplier condition) {
    m_event = new BooleanEvent(loop, condition);
  }

  /**
   * Copies the BooleanEvent into a Trigger object.
   *
   * @param toCast the BooleanEvent
   * @return a Trigger wrapping the given BooleanEvent
   * @see BooleanEvent#castTo(BiFunction)
   */
  public static Trigger cast(BooleanEvent toCast) {
    return toCast.castTo(Trigger::new);
  }

  /**
   * Creates a new trigger based on the given condition.
   *
   * <p>Polled by the default scheduler button loop.
   *
   * @param condition the condition represented by this trigger
   */
  public Trigger(BooleanSupplier condition) {
    this(CommandScheduler.getInstance().getDefaultButtonLoop(), condition);
  }

  /** Creates a new trigger that is always `false`. */
  @Deprecated
  public Trigger() {
    this(() -> false);
  }

  /**
   * Starts the given command whenever the condition changes from `false` to `true`.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   * @see #rising()
   */
  public Trigger onTrue(Command command) {
    requireNonNullParam(command, "command", "onRising");
    m_event.rising().ifHigh(command::schedule);
    return this;
  }

  /**
   * Starts the given command whenever the condition changes from `true` to `false`.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   * @see #falling()
   */
  public Trigger onFalse(Command command) {
    requireNonNullParam(command, "command", "onFalling");
    m_event.falling().ifHigh(command::schedule);
    return this;
  }

  /**
   * Starts the given command when the condition changes to `true` and cancels it when the condition
   * changes to `false`.
   *
   * <p>Doesn't re-start the command if it ends while the condition is still `true`. If the command
   * should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   */
  public Trigger whileTrue(Command command) {
    requireNonNullParam(command, "command", "whileHigh");
    m_event.rising().ifHigh(command::schedule);
    m_event.falling().ifHigh(command::cancel);
    return this;
  }

  /**
   * Starts the given command when the condition changes to `false` and cancels it when the
   * condition changes to `true`.
   *
   * <p>Doesn't re-start the command if it ends while the condition is still `false`. If the command
   * should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   */
  public Trigger whileFalse(Command command) {
    requireNonNullParam(command, "command", "whileLow");
    m_event.falling().ifHigh(command::schedule);
    m_event.rising().ifHigh(command::cancel);
    return this;
  }

  /**
   * Toggles a command when the condition changes from `false` to `true`.
   *
   * @param command the command to toggle
   * @return this trigger, so calls can be chained
   */
  public Trigger toggleOnTrue(Command command) {
    requireNonNullParam(command, "command", "toggleOnRising");
    m_event
        .rising()
        .ifHigh(
            () -> {
              if (!command.isScheduled()) {
                command.schedule();
              } else {
                command.cancel();
              }
            });
    return this;
  }

  /**
   * Toggles a command when the condition changes from `true` to the low state.
   *
   * @param command the command to toggle
   * @return this trigger, so calls can be chained
   */
  public Trigger toggleOnFalse(Command command) {
    requireNonNullParam(command, "command", "toggleOnFalling");
    m_event
        .falling()
        .ifHigh(
            () -> {
              if (!command.isScheduled()) {
                command.schedule();
              } else {
                command.cancel();
              }
            });
    return this;
  }

  /**
   * Starts the given command whenever the trigger just becomes active.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   * @deprecated Use {@link #onTrue(Command)} instead.
   */
  @Deprecated
  public Trigger whenActive(final Command command) {
    requireNonNullParam(command, "command", "whenActive");

    m_event.rising().ifHigh(command::schedule);
    return this;
  }

  /**
   * Runs the given runnable whenever the trigger just becomes active.
   *
   * @param toRun the runnable to run
   * @param requirements the required subsystems
   * @return this trigger, so calls can be chained
   * @deprecated Replace with {@link #onTrue(Command)}, creating the InstantCommand manually
   */
  @Deprecated
  public Trigger whenActive(final Runnable toRun, Subsystem... requirements) {
    return whenActive(new InstantCommand(toRun, requirements));
  }

  /**
   * Constantly starts the given command while the button is held.
   *
   * <p>{@link Command#schedule()} will be called repeatedly while the trigger is active, and will
   * be canceled when the trigger becomes inactive.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   * @deprecated Use {@link #whileTrue(Command)} with {@link
   *     edu.wpi.first.wpilibj2.command.RepeatCommand RepeatCommand}, or bind {@link
   *     Command#schedule() command::schedule} to {@link BooleanEvent#ifHigh(Runnable)} (passing no
   *     requirements).
   */
  @Deprecated
  public Trigger whileActiveContinuous(final Command command) {
    requireNonNullParam(command, "command", "whileActiveContinuous");

    m_event.ifHigh(command::schedule);
    m_event.falling().ifHigh(command::cancel);

    return this;
  }

  /**
   * Constantly runs the given runnable while the button is held.
   *
   * @param toRun the runnable to run
   * @param requirements the required subsystems
   * @return this trigger, so calls can be chained
   * @deprecated Use {@link #whileTrue(Command)} and construct a RunCommand manually
   */
  @Deprecated
  public Trigger whileActiveContinuous(final Runnable toRun, Subsystem... requirements) {
    return whileActiveContinuous(new InstantCommand(toRun, requirements));
  }

  /**
   * Starts the given command when the trigger initially becomes active, and ends it when it becomes
   * inactive, but does not re-start it in-between.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   * @deprecated Use {@link #whileTrue(Command)} instead.
   */
  @Deprecated
  public Trigger whileActiveOnce(final Command command) {
    requireNonNullParam(command, "command", "whileActiveOnce");

    m_event.rising().ifHigh(command::schedule);
    m_event.falling().ifHigh(command::cancel);

    return this;
  }

  /**
   * Starts the command when the trigger becomes inactive.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   * @deprecated Use {@link #onFalse(Command)} instead.
   */
  @Deprecated
  public Trigger whenInactive(final Command command) {
    requireNonNullParam(command, "command", "whenInactive");

    m_event.falling().ifHigh(command::schedule);

    return this;
  }

  /**
   * Runs the given runnable when the trigger becomes inactive.
   *
   * @param toRun the runnable to run
   * @param requirements the required subsystems
   * @return this trigger, so calls can be chained
   * @deprecated Construct the InstantCommand manually and replace with {@link #onFalse(Command)}
   */
  @Deprecated
  public Trigger whenInactive(final Runnable toRun, Subsystem... requirements) {
    return whenInactive(new InstantCommand(toRun, requirements));
  }

  /**
   * Toggles a command when the trigger becomes active.
   *
   * @param command the command to toggle
   * @return this trigger, so calls can be chained
   * @deprecated Use {@link #toggleOnTrue(Command)} instead.
   */
  @Deprecated
  public Trigger toggleWhenActive(final Command command) {
    requireNonNullParam(command, "command", "toggleWhenActive");

    m_event
        .rising()
        .ifHigh(
            () -> {
              if (command.isScheduled()) {
                command.cancel();
              } else {
                command.schedule();
              }
            });

    return this;
  }

  /**
   * Cancels a command when the trigger becomes active.
   *
   * @param command the command to cancel
   * @return this trigger, so calls can be chained
   * @deprecated Instead, pass {@link #rising()} as an end condition to {@link
   *     Command#until(BooleanSupplier)}.
   */
  @Deprecated
  public Trigger cancelWhenActive(final Command command) {
    requireNonNullParam(command, "command", "cancelWhenActive");

    m_event.rising().ifHigh(command::cancel);

    return this;
  }

  /**
   * Get the wrapped BooleanEvent.
   *
   * @return the wrapped BooleanEvent instance.
   */
  public BooleanEvent getEvent() {
    return m_event;
  }

  @Override
  public boolean getAsBoolean() {
    return m_event.getAsBoolean();
  }

  public Trigger and(BooleanSupplier trigger) {
    return cast(m_event.and(trigger));
  }

  public Trigger or(BooleanSupplier trigger) {
    return cast(m_event.or(trigger));
  }

  public Trigger negate() {
    return cast(m_event.negate());
  }

  public Trigger debounce(double seconds) {
    return debounce(seconds, Debouncer.DebounceType.kRising);
  }

  public Trigger debounce(double seconds, Debouncer.DebounceType type) {
    return cast(m_event.debounce(seconds, type));
  }

  public Trigger rising() {
    return cast(m_event.rising());
  }

  public Trigger falling() {
    return cast(m_event.falling());
  }
}
