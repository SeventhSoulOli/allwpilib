package edu.wpi.first.wpilibj.commandsv3;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;
import java.util.Objects;

public final class ContinuationScope {
  // The underlying jdk.internal.vm.ContinuationScope object
  final Object continuationScope;

  static final Class<?> jdk_internal_vm_ContinuationScope;
  private static final MethodHandle CONSTRUCTOR;

  static {
    try {
      jdk_internal_vm_ContinuationScope = Class.forName("jdk.internal.vm.ContinuationScope");

      var lookup =
          MethodHandles.privateLookupIn(jdk_internal_vm_ContinuationScope, MethodHandles.lookup());

      CONSTRUCTOR =
          lookup.findConstructor(
              jdk_internal_vm_ContinuationScope, MethodType.methodType(void.class, String.class));
    } catch (Throwable t) {
      throw new ExceptionInInitializerError(t);
    }
  }

  public ContinuationScope(String name) {
    Objects.requireNonNull(name);
    try {
      this.continuationScope = CONSTRUCTOR.invoke(name);
    } catch (Throwable e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public String toString() {
    return continuationScope.toString();
  }
}
