package edu.wpi.first.wpilibj3.command.async;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;
import java.lang.invoke.WrongMethodTypeException;

/** A wrapper around the JDK internal Continuation class. */
public final class Continuation {
  // The underlying jdk.internal.vm.Continuation object
  final Object continuation;

  static final Class<?> jdk_internal_vm_Continuation;
  private static final MethodHandle CONSTRUCTOR;
  private static final MethodHandle YIELD;
  private static final MethodHandle RUN;
  private static final MethodHandle IS_DONE;

  private static final MethodHandle java_lang_thread_setContinuation;

  static {
    try {
      jdk_internal_vm_Continuation = Class.forName("jdk.internal.vm.Continuation");

      var lookup =
          MethodHandles.privateLookupIn(jdk_internal_vm_Continuation, MethodHandles.lookup());

      CONSTRUCTOR =
          lookup.findConstructor(
              jdk_internal_vm_Continuation,
              MethodType.methodType(
                  void.class, ContinuationScope.jdk_internal_vm_ContinuationScope, Runnable.class));

      YIELD =
          lookup.findStatic(
              jdk_internal_vm_Continuation,
              "yield",
              MethodType.methodType(
                  boolean.class, ContinuationScope.jdk_internal_vm_ContinuationScope));

      RUN =
          lookup.findVirtual(
              jdk_internal_vm_Continuation, "run", MethodType.methodType(void.class));

      IS_DONE =
          lookup.findVirtual(
              jdk_internal_vm_Continuation, "isDone", MethodType.methodType(boolean.class));
    } catch (Throwable t) {
      throw new ExceptionInInitializerError(t);
    }
  }

  static {
    try {
      var lookup = MethodHandles.privateLookupIn(Thread.class, MethodHandles.lookup());

      java_lang_thread_setContinuation =
          lookup.findVirtual(
              Thread.class,
              "setContinuation",
              MethodType.methodType(void.class, Continuation.jdk_internal_vm_Continuation));
    } catch (Throwable t) {
      throw new ExceptionInInitializerError(t);
    }
  }

  /**
   * Constructs a continuation.
   *
   * @param scope the continuation's scope, used in yield
   * @param target the continuation's body
   */
  public Continuation(ContinuationScope scope, Runnable target) {
    try {
      this.continuation = CONSTRUCTOR.invoke(scope.continuationScope, target);
    } catch (Throwable t) {
      throw new RuntimeException(t);
    }
  }

  /**
   * Suspends the current continuations up to the given scope.
   *
   * @param scope The {@link ContinuationScope} to suspend
   * @return {@code true} for success; {@code false} for failure
   * @throws IllegalStateException if not currently in the given {@code scope},
   */
  public static boolean yield(ContinuationScope scope) {
    try {
      return (boolean) YIELD.invoke(scope.continuationScope);
    } catch (Throwable t) {
      throw new RuntimeException(t);
    }
  }

  /**
   * Mounts and runs the continuation body. If suspended, continues it from the last suspend point.
   */
  public void run() {
    try {
      RUN.invoke(continuation);
    } catch (WrongMethodTypeException | ClassCastException e) {
      throw new IllegalStateException("Unable to run the underlying continuation!", e);
    } catch (RuntimeException e) {
      // The bound task threw an exception; re-throw it
      throw e;
    } catch (Throwable t) {
      // Other error type (eg StackOverflowError, OutOfMemoryError), wrap in an Error so it won't
      // be caught by a naive `catch (Exception e)` statement
      throw new Error(t);
    }
  }

  /**
   * Tests whether this continuation is completed
   *
   * @return whether this continuation is completed
   */
  public boolean isDone() {
    try {
      return (boolean) IS_DONE.invoke(continuation);
    } catch (Throwable t) {
      throw new RuntimeException(t);
    }
  }

  /**
   * Mounds a continuation to the current thread. Accepts null for clearing the currently mounted
   * continuation.
   *
   * @param continuation the continuation to mount
   */
  public static void mountContinuation(Continuation continuation) {
    try {
      if (continuation == null) {
        java_lang_thread_setContinuation.invoke(Thread.currentThread(), null);
      } else {
        java_lang_thread_setContinuation.invoke(Thread.currentThread(), continuation.continuation);
      }
    } catch (Throwable t) {
      // Anything thrown internally by Thread.setContinuation.
      // It only assigns to a field, no way to throw
      // However, if the invocation fails for some reason, we'll end up with an
      // IllegalStateException when attempting to run an unmounted continuation
    }
  }

  @Override
  public String toString() {
    return continuation.toString();
  }
}
