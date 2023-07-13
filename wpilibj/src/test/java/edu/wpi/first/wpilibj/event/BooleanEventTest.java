// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj.event;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class BooleanEventTest {
  @Test
  void testBinaryCompositions() {
    var loop = new EventLoop();

    var andCounter = new AtomicInteger(0);
    var orCounter = new AtomicInteger(0);

    assertEquals(0, andCounter.get());
    assertEquals(0, orCounter.get());

    new BooleanEvent(loop, () -> true).and(() -> false).ifHigh(andCounter::incrementAndGet);
    new BooleanEvent(loop, () -> true).or(() -> false).ifHigh(orCounter::incrementAndGet);

    loop.poll();

    assertEquals(0, andCounter.get());
    assertEquals(1, orCounter.get());
  }

  @Test
  void testBinaryCompositionLoopSemantics() {
    var loop1 = new EventLoop();
    var loop2 = new EventLoop();

    var counter1 = new AtomicInteger(0);
    var counter2 = new AtomicInteger(0);

    new BooleanEvent(loop1, () -> true)
        .and(new BooleanEvent(loop2, () -> true))
        .ifHigh(counter1::incrementAndGet);

    new BooleanEvent(loop2, () -> true)
        .and(new BooleanEvent(loop1, () -> true))
        .ifHigh(counter2::incrementAndGet);

    assertEquals(0, counter1.get());
    assertEquals(0, counter2.get());

    loop1.poll();

    assertEquals(1, counter1.get());
    assertEquals(0, counter2.get());

    loop2.poll();

    assertEquals(1, counter1.get());
    assertEquals(1, counter2.get());
  }

  @Test
  void testEdgeDecorators() {
    var bool = new AtomicBoolean(false);
    var counter = new AtomicInteger(0);

    var loop = new EventLoop();

    new BooleanEvent(loop, bool::get).falling().ifHigh(counter::decrementAndGet);
    new BooleanEvent(loop, bool::get).rising().ifHigh(counter::incrementAndGet);

    assertEquals(0, counter.get());

    bool.set(false);
    loop.poll();

    assertEquals(0, counter.get());

    bool.set(true);
    loop.poll();

    assertEquals(1, counter.get());

    bool.set(true);
    loop.poll();

    assertEquals(1, counter.get());

    bool.set(false);
    loop.poll();

    assertEquals(0, counter.get());
  }

  @Test
  void testEdgeReuse() {
    var loop = new EventLoop();
    var bool = new AtomicBoolean(false);
    var counter = new AtomicInteger(0);

    var event = new BooleanEvent(loop, bool::get).rising();
    event.ifHigh(counter::incrementAndGet);
    event.ifHigh(counter::incrementAndGet);

    assertEquals(0, counter.get());

    loop.poll();

    assertEquals(0, counter.get());

    bool.set(true);
    loop.poll();

    assertEquals(2, counter.get());

    loop.poll();

    assertEquals(2, counter.get());

    bool.set(false);
    loop.poll();

    assertEquals(2, counter.get());

    bool.set(true);
    loop.poll();

    assertEquals(4, counter.get());
  }

  @Test
  void testEdgeReconstruct() {
    var loop = new EventLoop();
    var bool = new AtomicBoolean(false);
    var counter = new AtomicInteger(0);

    var event = new BooleanEvent(loop, bool::get);
    event.rising().ifHigh(counter::incrementAndGet);
    event.rising().ifHigh(counter::incrementAndGet);

    assertEquals(0, counter.get());

    loop.poll();

    assertEquals(0, counter.get());

    bool.set(true);
    loop.poll();

    assertEquals(2, counter.get());

    loop.poll();

    assertEquals(2, counter.get());

    bool.set(false);
    loop.poll();

    assertEquals(2, counter.get());

    bool.set(true);
    loop.poll();

    assertEquals(4, counter.get());
  }

  @Test
  void testMidLoopBooleanChange() {
    var loop = new EventLoop();
    var bool = new AtomicBoolean(false);
    var counter = new AtomicInteger(0);

    var event = new BooleanEvent(loop, bool::get).rising();
    event.ifHigh(
        () -> {
          bool.set(false);
          counter.incrementAndGet();
        });
    event.ifHigh(counter::incrementAndGet);

    assertEquals(0, counter.get());

    loop.poll();

    assertEquals(0, counter.get());

    bool.set(true);
    loop.poll();

    assertEquals(2, counter.get());

    loop.poll();

    assertEquals(2, counter.get());

    bool.set(false);
    loop.poll();

    assertEquals(2, counter.get());

    bool.set(true);
    loop.poll();

    assertEquals(4, counter.get());
  }

  @Test
  void testEventReuse() {
    var loop = new EventLoop();
    var bool1 = new AtomicBoolean(false);
    var bool2 = new AtomicBoolean(false);
    var bool3 = new AtomicBoolean(false);
    var counter = new AtomicInteger(0);

    var event1 = new BooleanEvent(loop, bool1::get).rising();
    var event2 = new BooleanEvent(loop, bool2::get).rising();
    var event3 = new BooleanEvent(loop, bool3::get).rising();
    event1.ifHigh(
        () -> {
          bool1.set(false);
          counter.incrementAndGet();
        });
    event1
        .and(event2)
        .ifHigh(
            () -> {
              bool3.set(false);
              counter.incrementAndGet();
            });
    event1
        .and(event3)
        .ifHigh(
            () -> {
              bool2.set(false);
              counter.incrementAndGet();
            });

    assertEquals(0, counter.get());

    bool1.set(true);
    bool2.set(true);
    bool3.set(true);
    loop.poll();

    assertEquals(3, counter.get());

    bool1.set(true);
    loop.poll();

    assertEquals(4, counter.get());

    bool2.set(true);
    bool3.set(true);
    loop.poll();

    assertEquals(4, counter.get());

    bool1.set(true);
    loop.poll();

    assertEquals(7, counter.get());

    bool1.set(true);
    bool2.set(true);
    loop.poll();

    assertEquals(9, counter.get());
  }
}
