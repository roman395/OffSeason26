package org.firstinspires.ftc.teamcode.opmodes.base;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import java.util.Objects;

/**
 * Общая основа для автономных OpMode.
 * <p>
 * Конкретный Auto должен:
 * - указать стартовую позицию;
 * - построить пути;
 * - вернуть одну общую Ivy-команду.
 */
public abstract class BaseAutoOpMode extends BaseRobotOpMode {
  
  private Command autoRoutine;
  private boolean started;
  
  @Override
  protected final void onRobotInit() {
    started = false;
    
    Pose startingPose = Objects.requireNonNull(
        getStartingPose(),
        "starting pose cannot be null"
    );
    
    robot.drive.setStartingPose(startingPose);
    
    /*
     * Сначала создаём все PathChain,
     * затем собираем из них общую команду.
     */
    buildPaths();
    
    autoRoutine = Objects.requireNonNull(
        buildAutoRoutine(),
        "auto routine cannot be null"
    );
    
    onAutoInit();
  }
  
  @Override
  protected final void onRobotInitLoop() {
    onAutoInitLoop();
  }
  
  @Override
  protected final void onRobotStart() {
    started = true;
    
    /*
     * Весь Auto запускается одной командой.
     */
    onAutoStart();
    
    Scheduler.schedule(autoRoutine);
    
  }
  
  @Override
  protected final void onRobotLoop(double dtSeconds) {
    onAutoLoop(dtSeconds);
  }
  
  @Override
  protected final void onRobotTelemetry() {
    Pose pose = robot.drive.getPose();
    
    telemetry.addData("Auto", getAutoStatus());
    telemetry.addData("X", "%.2f", pose.getX());
    telemetry.addData("Y", "%.2f", pose.getY());
    telemetry.addData(
        "Heading",
        "%.1f deg",
        Math.toDegrees(pose.getHeading())
    );
    
    onAutoTelemetry();
  }
  
  /**
   * Позиция робота на поле перед началом Auto.
   */
  protected abstract Pose getStartingPose();
  
  /**
   * Здесь создаются все PathChain.
   */
  protected abstract void buildPaths();
  
  /**
   * Возвращает законченную композицию Ivy-команд.
   */
  protected abstract Command buildAutoRoutine();
  
  /*
   * Необязательные хуки для конкретных Auto.
   */
  
  protected void onAutoInit() {
  }
  
  protected void onAutoInitLoop() {
  }
  
  protected void onAutoStart() {
  }
  
  protected void onAutoLoop(double dtSeconds) {
  }
  
  protected void onAutoTelemetry() {
  }
  
  protected final boolean isAutoFinished() {
    return started && !autoRoutine.isScheduled();
  }
  
  private String getAutoStatus() {
    if (!started) {
      return "READY";
    }
    
    return autoRoutine.isScheduled()
        ? "RUNNING"
        : "FINISHED";
  }
  
}