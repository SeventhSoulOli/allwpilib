// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj2.command;

import java.util.Set;
import java.util.function.Supplier;

/**
 * `Resource` is the <a href="https://en.wikipedia.org/wiki/Lock_(computer_science)">locking</a> 
 * implementation for the Command-based concurrency model, encapsulating program state that should 
 * only be controlled by one {@link Command} at a time.  If multiple commands requiring the same
 * resource are scheduled at the same time, the {@link CommandScheduler} will resolve the resource
 * contention by interrupting (or refusing to schedule) all but one of the commands.
 * 
 * <p>Typically, protected state represents hardware resources (like motors) for which control
 * disputes would present an operational hazard.  But, it may also be appropriate to
 * protect software resources, like vision pipelines, whose outputs might become invalid if
 * their configurations are changed inappropriately during a task.
 * 
 * <p>Classes containing protected state should implement this interface.  Commands that require
 * said state should be written using the Command factories (e.g. {@link Resource#run(Runnable)}
 * defined on this interface, which automatically declare the resource requirement.
 */
public interface Resource {
  /**
   * Gets the name of this resource for telemetry.
   *
   * @return resource name
   */
  default String getName() {
    return this.getClass().getSimpleName();
  }

  /**
   * Returns the command currently running on this resource. Returns null if no command is
   * currently scheduled that requires this resource.
   *
   * @return the scheduled command currently requiring this resource
   */
  default Command getCurrentCommand() {
    return CommandScheduler.getInstance().requiring(this);
  }

  /**
   * Constructs a command that runs an action once and finishes. Requires this resource.
   *
   * @param action the action to run
   * @return the command
   * @see InstantCommand
   */
  default Command runOnce(Runnable action) {
    return Commands.runOnce(action, this);
  }

  /**
   * Constructs a command that runs an action every iteration until interrupted. Requires this
   * resource.
   *
   * @param action the action to run
   * @return the command
   * @see RunCommand
   */
  default Command run(Runnable action) {
    return Commands.run(action, this);
  }

  /**
   * Constructs a command that runs an action once and another action when the command is
   * interrupted. Requires this resource.
   *
   * @param start the action to run on start
   * @param end the action to run on interrupt
   * @return the command
   * @see StartEndCommand
   */
  default Command startEnd(Runnable start, Runnable end) {
    return Commands.startEnd(start, end, this);
  }

  /**
   * Constructs a command that runs an action every iteration until interrupted, and then runs a
   * second action. Requires this resource.
   *
   * @param run the action to run every iteration
   * @param end the action to run on interrupt
   * @return the command
   */
  default Command runEnd(Runnable run, Runnable end) {
    return Commands.runEnd(run, end, this);
  }

  /**
   * Constructs a command that runs an action once and then runs another action every iteration
   * until interrupted. Requires this resource.
   *
   * @param start the action to run on start
   * @param run the action to run every iteration
   * @return the command
   */
  default Command startRun(Runnable start, Runnable run) {
    return Commands.startRun(start, run, this);
  }

  /**
   * Constructs a {@link DeferredCommand} with the provided supplier. This resource is added as a
   * requirement.
   *
   * @param supplier the command supplier.
   * @return the command.
   * @see DeferredCommand
   */
  default Command defer(Supplier<Command> supplier) {
    return Commands.defer(supplier, Set.of(this));
  }
}
