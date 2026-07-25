package org.firstinspires.ftc.teamcode.robot.core;

/**
 * Базовая реализация подсистемы.
 *
 * Управляет:
 * - жизненным циклом;
 * - включением и отключением;
 * - аварийным состоянием;
 * - безопасной остановкой выходов.
 */
public abstract class AbstractSubsystem implements Subsystem {
  
  private enum LifecycleState {
    NEW,
    INITIALIZED,
    RUNNING,
    STOPPED
  }
  
  private final String name;
  
  private LifecycleState state = LifecycleState.NEW;
  private boolean enabled = true;
  private String faultMessage = "";
  
  protected AbstractSubsystem(String name) {
    if (name == null || name.trim().isEmpty()) {
      throw new IllegalArgumentException(
          "Subsystem name cannot be empty"
      );
    }
    
    this.name = name;
  }
  
  @Override
  public final String getName() {
    return name;
  }
  
  @Override
  public final void init() {
    if (state != LifecycleState.NEW) {
      throw new IllegalStateException(
          name + " cannot be initialized from state " + state
      );
    }
    
    try {
      onInit();
      state = LifecycleState.INITIALIZED;
    } catch (RuntimeException exception) {
      enterFault("Initialization failed", exception);
      throw exception;
    }
  }
  
  @Override
  public final void start() {
    if (state != LifecycleState.INITIALIZED) {
      throw new IllegalStateException(
          name + " cannot be started from state " + state
      );
    }
    
    if (isFaulted()) {
      throw new IllegalStateException(
          name + " cannot start: " + faultMessage
      );
    }
    
    try {
      onStart();
      state = LifecycleState.RUNNING;
    } catch (RuntimeException exception) {
      enterFault("Start failed", exception);
      throw exception;
    }
  }
  
  @Override
  public final void periodic(double dtSeconds) {
    if (state != LifecycleState.RUNNING) {
      return;
    }
    
    if (!enabled || isFaulted()) {
      return;
    }
    
    if (dtSeconds < 0
        || Double.isNaN(dtSeconds)
        || Double.isInfinite(dtSeconds)) {
      
      IllegalArgumentException exception =
          new IllegalArgumentException(
              "Invalid dtSeconds: " + dtSeconds
          );
      
      enterFault("Invalid loop time", exception);
      throw exception;
    }
    
    try {
      onPeriodic(dtSeconds);
    } catch (RuntimeException exception) {
      enterFault("Periodic update failed", exception);
      throw exception;
    }
  }
  
  @Override
  public final void stop() {
    if (state == LifecycleState.STOPPED) {
      return;
    }
    
    try {
      onStop();
    } finally {
      state = LifecycleState.STOPPED;
      enabled = false;
      stopOutputs();
    }
  }
  
  @Override
  public final boolean isEnabled() {
    return enabled;
  }
  
  @Override
  public final void setEnabled(boolean enabled) {
    if (enabled && isFaulted()) {
      throw new IllegalStateException(
          name + " cannot be enabled while faulted"
      );
    }
    
    if (this.enabled == enabled) {
      return;
    }
    
    this.enabled = enabled;
    
    if (!enabled) {
      stopOutputs();
    }
  }
  
  @Override
  public final boolean isFaulted() {
    return !faultMessage.isEmpty();
  }
  
  @Override
  public final String getFaultMessage() {
    return faultMessage;
  }
  
  /**
   * Переводит подсистему в аварийное состояние без остановки OpMode.
   *
   * После вызова fail() конкретная подсистема должна закончить
   * текущий метод через return.
   */
  protected final void fail(String message) {
    if (message == null || message.trim().isEmpty()) {
      message = "Unknown subsystem fault";
    }
    
    faultMessage = message;
    enabled = false;
    stopOutputs();
  }
  
  private void enterFault(
      String context,
      RuntimeException exception
  ) {
    String details = exception.getMessage();
    
    if (details == null || details.trim().isEmpty()) {
      details = exception.getClass().getSimpleName();
    }
    
    faultMessage = context + ": " + details;
    enabled = false;
    
    try {
      stopOutputs();
    } catch (RuntimeException stopException) {
      exception.addSuppressed(stopException);
    }
  }
  
  /*
   * Методы-шаблоны для конкретных подсистем.
   */
  
  protected void onInit() {
  }
  
  protected void onStart() {
  }
  
  protected abstract void onPeriodic(double dtSeconds);
  
  protected void onStop() {
  }
  
  /**
   * Немедленно переводит исполнительные устройства
   * в безопасное состояние.
   */
  protected abstract void stopOutputs();
}