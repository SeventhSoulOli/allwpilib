package edu.wpi.first.wpilibj.commandsv3;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

@Timeout(5)
class SchedulerTest {
  private Scheduler scheduler;

  @BeforeEach
  void setup() {
    scheduler = new Scheduler();
  }

  @Test
  void basic() {
    var enabled = new AtomicBoolean(false);
    var ran = new AtomicBoolean(false);
    var command =
        Command.noRequirements(
                (coroutine) -> {
                  do {
                    coroutine.yield();
                  } while (!enabled.get());
                  ran.set(true);
                })
            .named("Basic Command");

    scheduler.schedule(command);
    scheduler.run();
    assertTrue(scheduler.isRunning(command), "Command should be running after being scheduled");

    enabled.set(true);
    scheduler.run();
    if (scheduler.isRunning(command)) {
      fail("Command should no longer be running after awaiting its completion");
    }

    assertTrue(ran.get());
  }

  @Test
  void higherPriorityCancels() {
    var subsystem = new RequireableResource("Subsystem", scheduler);

    var lower = new PriorityCommand(-1000, subsystem);
    var higher = new PriorityCommand(+1000, subsystem);

    scheduler.schedule(lower);
    scheduler.run();
    assertTrue(scheduler.isRunning(lower));

    scheduler.schedule(higher);
    scheduler.run();
    assertTrue(scheduler.isRunning(higher));
    assertFalse(scheduler.isRunning(lower));
  }

  @Test
  void lowerPriorityDoesNotCancel() {
    var subsystem = new RequireableResource("Subsystem", scheduler);

    var lower = new PriorityCommand(-1000, subsystem);
    var higher = new PriorityCommand(+1000, subsystem);

    scheduler.schedule(higher);
    scheduler.run();
    assertTrue(scheduler.isRunning(higher));

    scheduler.schedule(lower);
    scheduler.run();
    if (!scheduler.isRunning(higher)) {
      fail("Higher priority command should still be running");
    }
    if (scheduler.isRunning(lower)) {
      fail("Lower priority command should not be running");
    }
  }

  @Test
  void samePriorityCancels() {
    var subsystem = new RequireableResource("Subsystem", scheduler);

    var first = new PriorityCommand(512, subsystem);
    var second = new PriorityCommand(512, subsystem);

    scheduler.schedule(first);
    scheduler.run();
    assertTrue(scheduler.isRunning(first));

    scheduler.schedule(second);
    scheduler.run();
    assertTrue(scheduler.isRunning(second), "New command should be running");
    assertFalse(scheduler.isRunning(first), "Old command should be canceled");
  }

  @Test
  void atomicity() {
    var resource =
        new RequireableResource("X", scheduler) {
          int x = 0;
        };

    // Launch 100 commands that each call `x++` 500 times.
    // If commands run on different threads, the lack of atomic
    // operations or locks will mean the final number will be
    // less than the expected 50,000
    int numCommands = 100;
    int iterations = 500;

    for (int cmdCount = 0; cmdCount < numCommands; cmdCount++) {
      var command =
          Command.noRequirements(
                  (coroutine) -> {
                    for (int i = 0; i < iterations; i++) {
                      coroutine.yield();
                      resource.x++;
                    }
                  })
              .named("CountCommand[" + cmdCount + "]");

      scheduler.schedule(command);
    }

    for (int i = 0; i < iterations; i++) {
      scheduler.run();
    }
    scheduler.run();

    assertEquals(numCommands * iterations, resource.x);
  }

  @Test
  void errorDetection() {
    var resource = new RequireableResource("X", scheduler);

    var command =
        resource
            .run(
                (coroutine) -> {
                  throw new RuntimeException("The exception");
                })
            .named("Bad Behavior");

    scheduler.schedule(command);

    try {
      scheduler.run();
      fail("An exception should have been thrown");
    } catch (CommandExecutionException e) {
      if (e.getCommand() != command) {
        fail("Expected command " + command + ", but was " + e.getCommand());
      }

      var cause = e.getCause();
      if (cause instanceof RuntimeException re) {
        assertEquals("The exception", re.getMessage());
      } else {
        fail(
            "Expected cause to be a RuntimeException with message 'The exception', but was "
                + cause);
      }
    } catch (Throwable t) {
      fail("Expected a CommandExecutionException to be thrown, but got " + t);
    }
  }

  @Test
  void runResource() {
    var example =
        new RequireableResource("Counting", scheduler) {
          int x = 0;
        };

    Command countToTen =
        example
            .run(
                (coroutine) -> {
                  example.x = 0;
                  for (int i = 0; i < 10; i++) {
                    coroutine.yield();
                    example.x++;
                  }
                })
            .named("Count To Ten");

    scheduler.schedule(countToTen);
    for (int i = 0; i < 10; i++) {
      scheduler.run();
    }
    scheduler.run();

    assertEquals(10, example.x);
  }

  @Test
  void cancelOnInterruptDoesNotResume() {
    var count = new AtomicInteger(0);

    var resource = new RequireableResource("Resource", scheduler);

    var interrupter =
        Command.requiring(resource)
            .executing((coroutine) -> {})
            .withPriority(2)
            .named("Interrupter");

    var canceledCommand =
        Command.requiring(resource)
            .executing(
                (coroutine) -> {
                  count.set(1);
                  coroutine.yield();
                  count.set(2);
                })
            .withPriority(1)
            .named("Cancel By Default");

    scheduler.schedule(canceledCommand);
    scheduler.run();

    scheduler.schedule(interrupter);
    scheduler.run();
    assertEquals(1, count.get()); // the second "set" call should not have run
  }

  @Test
  void scheduleOverDefaultDoesNotRescheduleDefault() {
    var count = new AtomicInteger(0);

    var resource = new RequireableResource("Resource", scheduler);
    var defaultCmd =
        resource
            .run(
                (coroutine) -> {
                  while (true) {
                    count.incrementAndGet();
                    coroutine.yield();
                  }
                })
            .suspendingOnInterrupt()
            .withPriority(-1)
            .named("Default Command");

    var newerCmd = resource.run((coroutine) -> {}).named("Newer Command");
    resource.setDefaultCommand(defaultCmd);
    scheduler.run();
    assertTrue(scheduler.isRunning(defaultCmd), "Default command should be running");

    scheduler.schedule(newerCmd);
    scheduler.run();
    assertFalse(scheduler.isRunning(defaultCmd), "Default command should have been interrupted");
    assertEquals(1, count.get(), "Default command should have run once");

    scheduler.run();
    assertTrue(scheduler.isRunning(defaultCmd));
  }

  @Test
  void cancelAllCancelsAll() {
    var commands = new ArrayList<Command>(10);
    for (int i = 1; i <= 10; i++) {
      commands.add(Command.noRequirements(Coroutine::yield).named("Command " + i));
    }
    commands.forEach(scheduler::schedule);
    scheduler.run();
    scheduler.cancelAll();
    for (Command command : commands) {
      if (scheduler.isRunning(command)) {
        fail(command.name() + " was not canceled by cancelAll()");
      }
    }
  }

  @Test
  void cancelAllStartsDefaults() {
    var resources = new ArrayList<RequireableResource>(10);
    for (int i = 1; i <= 10; i++) {
      resources.add(new RequireableResource("System " + i, scheduler));
    }

    var command =
        new CommandBuilder().requiring(resources).executing(Coroutine::yield).named("Big Command");

    // Scheduling the command should evict the on-deck default commands
    scheduler.schedule(command);
    // Then running should get it into the set of running commands
    scheduler.run();
    // Cancelling should clear out the set of running commands
    scheduler.cancelAll();
    // Then ticking the scheduler once to fully remove the command and schedule the defaults
    scheduler.run();
    // Then one more tick to start running the scheduled defaults
    scheduler.run();

    if (scheduler.isRunning(command)) {
      System.err.println(scheduler.getRunningCommands());
      fail(command.name() + " was not canceled by cancelAll()");
    }

    for (var resource : resources) {
      var runningCommands = scheduler.getRunningCommandsFor(resource);
      assertEquals(
          1,
          runningCommands.size(),
          "Resource " + resource + " should have exactly one running command");
      assertInstanceOf(
          IdleCommand.class,
          runningCommands.getFirst(),
          "Resource " + resource + " is not running the default command");
    }
  }

  @Test
  void nestedResourceNestedCommands() {
    var inner = new RequireableResource("Inner", scheduler);
    var outer =
        new RequireableResource("Outer", scheduler) {
          @Override
          public Set<RequireableResource> nestedResources() {
            return Set.of(inner);
          }
        };

    var innerRan = new AtomicBoolean(false);
    var outerRan = new AtomicBoolean(false);

    var innerCommand =
        inner
            .run(
                (coroutine) -> {
                  innerRan.set(true);
                })
            .named("Inner Command");

    var outerCommand =
        outer
            .run(
                (coroutine) -> {
                  coroutine.await(innerCommand);
                  outerRan.set(true);
                })
            .named("Outer Command");

    scheduler.schedule(outerCommand);
    // First run: schedules the inner command
    scheduler.run();

    // Second run: inner command sets the flag and exits
    scheduler.run();
    assertTrue(innerRan.get());
    assertFalse(scheduler.isRunning(innerCommand));

    // Third run: outer command sets the flag and exits
    scheduler.run();
    assertTrue(outerRan.get());
    assertFalse(scheduler.isRunning(outerCommand));
  }

  @Test
  void cancelDeeplyNestedCompositions() {
    Command root = Command.noRequirements((co) -> {
      co.await(Command.noRequirements((co2) -> {
        co2.await(Command.noRequirements((co3) -> {
          co3.await(Command.noRequirements(Coroutine::park).named("Park"));
        }).named("C3"));
      }).named("C2"));
    }).named("Root");

    scheduler.schedule(root);

    scheduler.run(); // schedule c2
    scheduler.run(); // schedule c3
    scheduler.run(); // schedule park
    scheduler.run(); // start park
    assertEquals(4, scheduler.getRunningCommands().size());

    scheduler.cancel(root);
    assertEquals(0, scheduler.getRunningCommands().size());
  }

  @Test
  void compositionsDoNotSelfCancel() {
    var res = new RequireableResource("The Resource", scheduler);
    var group = res.run((co) -> {
      co.await(res.run((co2) -> {
        co2.await(res.run((co3) -> {
          co3.await(res.run(Coroutine::park).named("Park"));
        }).named("C3"));
      }).named("C2"));
    }).named("Group");

    scheduler.schedule(group);
    scheduler.run(); // schedule c2
    scheduler.run(); // schedule c3
    scheduler.run(); // schedule park
    scheduler.run(); // start park
    assertEquals(4, scheduler.getRunningCommands().size());
    assertTrue(scheduler.isRunning(group));
  }

  @Test
  void compositionsDoNotNeedRequirements() {
    var r1 = new RequireableResource("R1", scheduler);
    var r2 = new RequireableResource("r2", scheduler);

    // the group has no requirements, but can schedule child commands that do
    var group = Command.noRequirements((co) -> {
      co.awaitAll(
          r1.run(Coroutine::park).named("R1 Command"),
          r2.run(Coroutine::park).named("R2 Command"));
    }).named("Composition");

    scheduler.schedule(group);
    scheduler.run(); // schedule r1 and r2 commands
    scheduler.run(); // start r1 and r2 commands
    assertEquals(3, scheduler.getRunningCommands().size());
  }

  @Test
  void compositionsCannotAwaitConflictingCommands() {
    var res = new RequireableResource("The Resource", scheduler);

    var group = Command.noRequirements((co) -> {
      co.awaitAll(
          res.run(Coroutine::park).named("First"),
          res.run(Coroutine::park).named("Second")
      );
    }).named("Group");

    scheduler.schedule(group);

    // Running should attempt to schedule multiple conflicting commands
    try {
      scheduler.run();
      fail("An exception should have been thrown");
    } catch (CommandExecutionException e) {
      if (e.getCause() instanceof IllegalArgumentException iae) {
        assertEquals("Command Second requires resources that are already used by First. Both require The Resource", iae.getMessage());
      } else {
        fail("Unexpected exception: " + e);
      }
    }
  }

  @Test
  void doesNotRunOnCancelWhenInterruptingOnDeck() {
    var ran = new AtomicBoolean(false);

    var resource = new RequireableResource("The Resource", scheduler);
    var cmd = resource.run(Coroutine::yield).whenCanceled(() -> ran.set(true)).named("cmd");
    var interrupter = resource.run(Coroutine::yield).named("Interrupter");
    scheduler.schedule(cmd);
    scheduler.schedule(interrupter);
    scheduler.run();

    assertFalse(ran.get(), "onCancel ran when it shouldn't have!");
  }

  @Test
  void doesNotRunOnCancelWhenCancellingOnDeck() {
    var ran = new AtomicBoolean(false);

    var resource = new RequireableResource("The Resource", scheduler);
    var cmd = resource.run(Coroutine::yield).whenCanceled(() -> ran.set(true)).named("cmd");
    scheduler.schedule(cmd);
    // cancelling before calling .run()
    scheduler.cancel(cmd);
    scheduler.run();

    assertFalse(ran.get(), "onCancel ran when it shouldn't have!");
  }

  @Test
  void doesNotRunOnCancelWhenInterruptingCommand() {
    var ran = new AtomicBoolean(false);

    var resource = new RequireableResource("The Resource", scheduler);
    var cmd = resource.run(Coroutine::yield).whenCanceled(() -> ran.set(true)).named("cmd");
    var interrupter = resource.run(Coroutine::yield).named("Interrupter");
    scheduler.schedule(cmd);
    scheduler.run();
    scheduler.schedule(interrupter);

    assertFalse(ran.get(), "onCancel ran when it shouldn't have!");
  }

  @Test
  void doesNotRunOnCancelWhenCompleting() {
    var ran = new AtomicBoolean(false);

    var resource = new RequireableResource("The Resource", scheduler);
    var cmd = resource.run(Coroutine::yield).whenCanceled(() -> ran.set(true)).named("cmd");
    scheduler.schedule(cmd);
    scheduler.run();
    scheduler.run();

    assertFalse(scheduler.isScheduledOrRunning(cmd));
    assertFalse(ran.get(), "onCancel ran when it shouldn't have!");
  }

  @Test
  void runsOnCancelWhenCancelling() {
    var ran = new AtomicBoolean(false);

    var resource = new RequireableResource("The Resource", scheduler);
    var cmd = resource.run(Coroutine::yield).whenCanceled(() -> ran.set(true)).named("cmd");
    scheduler.schedule(cmd);
    scheduler.run();
    scheduler.cancel(cmd);

    assertTrue(ran.get(), "onCancel should have run!");
  }

  @Test
  void runsOnCancelWhenCancellingParent() {
    var ran = new AtomicBoolean(false);

    var resource = new RequireableResource("The Resource", scheduler);
    var cmd = resource.run(Coroutine::yield).whenCanceled(() -> ran.set(true)).named("cmd");

    var group = new Sequence("Seq", Collections.singletonList(cmd));
    scheduler.schedule(group);
    scheduler.run();
    scheduler.cancel(group);

    assertTrue(ran.get(), "onCancel should have run!");
  }

  record PriorityCommand(int priority, RequireableResource... subsystems) implements Command {
    @Override
    public void run(Coroutine coroutine) {
      coroutine.park();
    }

    @Override
    public Set<RequireableResource> requirements() {
      return Set.of(subsystems);
    }

    @Override
    public String name() {
      return toString();
    }

    @Override
    public String toString() {
      return "PriorityCommand[priority=" + priority + ", subsystems=" + Set.of(subsystems) + "]";
    }
  }
}
