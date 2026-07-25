package org.firstinspires.ftc.teamcode.opmodes.base;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.core.LoopClock;
import org.firstinspires.ftc.teamcode.robot.core.Robot;

/**
 * Общая основа для TeleOp, Autonomous и тестовых OpMode.
 *
 * Управляет:
 * - жизненным циклом Robot;
 * - временем цикла;
 * - Ivy Scheduler;
 * - безопасной остановкой.
 */
public abstract class BaseRobotOpMode extends OpMode {
  
  protected Robot robot;
  protected final LoopClock loopClock = new LoopClock();
  
  private boolean shutdown;
  
  @Override
  public final void init() {
    shutdown = false;
    
    /*
     * Scheduler статический, поэтому очищаем команды,
     * оставшиеся от предыдущего OpMode.
     */
    Scheduler.reset();
    
    robot = new Robot(hardwareMap);
    
    try {
      robot.init();
      onRobotInit();
      
      updateCoreTelemetry("INITIALIZED");
    } catch (RuntimeException exception) {
      shutdownAfterFailure(exception);
      throw exception;
    }
  }
  
  @Override
  public final void init_loop() {
    if (shutdown) {
      return;
    }
    
    try {
      onRobotInitLoop();
      updateCoreTelemetry("WAITING FOR START");
    } catch (RuntimeException exception) {
      shutdownAfterFailure(exception);
      throw exception;
    }
  }
  
  @Override
  public final void start() {
    if (shutdown) {
      return;
    }
    
    loopClock.reset();
    
    try {
      /*
       * Сначала запускаем подсистемы, затем пользовательский код.
       * Поэтому onRobotStart() уже может безопасно планировать команды.
       */
      robot.start();
      onRobotStart();
    } catch (RuntimeException exception) {
      shutdownAfterFailure(exception);
      throw exception;
    }
  }
  
  @Override
  public final void loop() {
    if (shutdown) {
      return;
    }
    
    double dtSeconds = loopClock.tick();
    
    try {
      /*
       * 1. Читаем управление и планируем команды.
       */
      onRobotLoop(dtSeconds);
      
      /*
       * 2. Ivy обновляет активные команды.
       */
      Scheduler.execute();
      
      /*
       * 3. Подсистемы применяют новые цели к оборудованию.
       */
      robot.periodic(dtSeconds);
      
      /*
       * 4. Собираем пользовательскую телеметрию.
       */
      onRobotTelemetry();
      
      updateCoreTelemetry("RUNNING");
    } catch (RuntimeException exception) {
      shutdownAfterFailure(exception);
      throw exception;
    }
  }
  
  @Override
  public final void stop() {
    shutdown();
  }
  
  /*
   * Методы, которые могут переопределять конкретные OpMode.
   */
  
  protected void onRobotInit() {
  }
  
  protected void onRobotInitLoop() {
  }
  
  protected void onRobotStart() {
  }
  
  protected void onRobotLoop(double dtSeconds) {
  }
  
  protected void onRobotTelemetry() {
  }
  
  protected void onRobotStop() {
  }
  
  private void updateCoreTelemetry(String status) {
    telemetry.addData("Robot status", status);
    
    if (loopClock.getRawDtSeconds() > 0) {
      telemetry.addData(
          "Loop",
          "%.1f Hz | %.1f ms",
          loopClock.getLoopFrequencyHz(),
          loopClock.getRawDtSeconds() * 1000
      );
    }
    
    telemetry.update();
  }
  
  private void shutdownAfterFailure(
      RuntimeException originalException
  ) {
    try {
      shutdown();
    } catch (RuntimeException shutdownException) {
      originalException.addSuppressed(shutdownException);
    }
  }
  
  private void shutdown() {
    if (shutdown) {
      return;
    }
    
    /*
     * Сразу ставим true, чтобы повторный вызов stop()
     * ничего не выполнял второй раз.
     */
    shutdown = true;
    
    RuntimeException firstException = null;
    
    try {
      Scheduler.reset();
    } catch (RuntimeException exception) {
      firstException = exception;
    }
    
    try {
      if (robot != null) {
        robot.stop();
      }
    } catch (RuntimeException exception) {
      firstException = appendException(
          firstException,
          exception
      );
    }
    
    try {
      onRobotStop();
    } catch (RuntimeException exception) {
      firstException = appendException(
          firstException,
          exception
      );
    }
    
    if (firstException != null) {
      throw firstException;
    }
  }
  
  private RuntimeException appendException(
      RuntimeException first,
      RuntimeException next
  ) {
    if (first == null) {
      return next;
    }
    
    first.addSuppressed(next);
    return first;
  }
}