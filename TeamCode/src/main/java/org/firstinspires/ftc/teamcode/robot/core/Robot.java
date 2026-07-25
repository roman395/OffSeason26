package org.firstinspires.ftc.teamcode.robot.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * Главный контейнер робота.
 *
 * Создаёт, хранит и обновляет все подсистемы в правильном порядке.
 */
public final class Robot {
  
  private enum LifecycleState {
    NEW,
    INITIALIZED,
    RUNNING,
    STOPPED
  }
  
  private final List<Subsystem> subsystems = new ArrayList<>();
  
  private LifecycleState state = LifecycleState.NEW;
  public final DriveSubsystem drive;
  public Robot(HardwareMap hardwareMap) {
    Objects.requireNonNull(hardwareMap, "hardwareMap");
    
    drive = register(new DriveSubsystem(Constants.createFollower(hardwareMap)));
    /*
     * Здесь позднее появится создание подсистем:
     *
     * drive = register(new DriveSubsystem(...));
     * lift = register(new LiftSubsystem(...));
     * intake = register(new IntakeSubsystem(...));
     */
  }
  
  /**
   * Добавляет подсистему в жизненный цикл робота и возвращает её
   * с сохранением конкретного типа.
   */
  private <T extends Subsystem> T register(T subsystem) {
    Objects.requireNonNull(subsystem, "subsystem");
    
    if (state != LifecycleState.NEW) {
      throw new IllegalStateException(
          "Subsystems can only be registered during Robot construction"
      );
    }
    
    for (Subsystem registered : subsystems) {
      if (registered.getName().equals(subsystem.getName())) {
        throw new IllegalArgumentException(
            "Duplicate subsystem name: " + subsystem.getName()
        );
      }
    }
    
    subsystems.add(subsystem);
    return subsystem;
  }
  
  /**
   * Инициализирует все зарегистрированные подсистемы.
   */
  public void init() {
    requireState(LifecycleState.NEW, "init");
    
    try {
      for (Subsystem subsystem : subsystems) {
        subsystem.init();
      }
      
      state = LifecycleState.INITIALIZED;
    } catch (RuntimeException exception) {
      stopAfterFailure(exception);
      throw exception;
    }
  }
  
  /**
   * Запускает все подсистемы после нажатия START.
   */
  public void start() {
    requireState(LifecycleState.INITIALIZED, "start");
    
    try {
      for (Subsystem subsystem : subsystems) {
        subsystem.start();
      }
      
      state = LifecycleState.RUNNING;
    } catch (RuntimeException exception) {
      stopAfterFailure(exception);
      throw exception;
    }
  }
  
  /**
   * Обновляет каждую подсистему один раз за цикл.
   */
  public void periodic(double dtSeconds) {
    if (state != LifecycleState.RUNNING) {
      return;
    }
    
    try {
      for (Subsystem subsystem : subsystems) {
        subsystem.periodic(dtSeconds);
      }
    } catch (RuntimeException exception) {
      stopAfterFailure(exception);
      throw exception;
    }
  }
  
  /**
   * Останавливает подсистемы в обратном порядке регистрации.
   */
  public void stop() {
    if (state == LifecycleState.STOPPED) {
      return;
    }
    
    RuntimeException firstException = null;
    
    for (int index = subsystems.size() - 1; index >= 0; index--) {
      try {
        subsystems.get(index).stop();
      } catch (RuntimeException exception) {
        if (firstException == null) {
          firstException = exception;
        } else {
          firstException.addSuppressed(exception);
        }
      }
    }
    
    state = LifecycleState.STOPPED;
    
    if (firstException != null) {
      throw firstException;
    }
  }
  
  /**
   * Возвращает список только для чтения.
   *
   * Позднее он понадобится менеджеру телеметрии.
   */
  public List<Subsystem> getSubsystems() {
    return Collections.unmodifiableList(subsystems);
  }
  
  private void requireState(
      LifecycleState expectedState,
      String operation
  ) {
    if (state != expectedState) {
      throw new IllegalStateException(
          "Cannot call Robot."
              + operation
              + "() while state is "
              + state
              + "; expected "
              + expectedState
      );
    }
  }
  
  /**
   * При неожиданной ошибке пытается остановить остальные подсистемы,
   * не скрывая исходное исключение.
   */
  private void stopAfterFailure(RuntimeException originalException) {
    try {
      stop();
    } catch (RuntimeException stopException) {
      originalException.addSuppressed(stopException);
    }
  }
}