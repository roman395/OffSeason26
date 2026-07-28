package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.base.BaseRobotOpMode;
import org.firstinspires.ftc.teamcode.robot.commands.drive.DriveCommands;
import org.firstinspires.ftc.teamcode.robot.commands.lift.LiftCommands;
import org.firstinspires.ftc.teamcode.robot.config.LiftConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftSubsystem;

@Configurable
@TeleOp(name = "Main TeleOp", group = "Competition")
public final class MainTeleOp extends BaseRobotOpMode {
  
  public static double DRIVE_DEADBAND = 0.05;
  public static double SLOW_MODE_MULTIPLIER = 0.35;
  
  private boolean slowMode;
  private boolean fieldCentric;
  
  private Command activeLiftPositionCommand;
  
  @Override
  protected void onRobotStart() {
    Scheduler.schedule(
        DriveCommands.teleOpDrive(
            robot.drive,
            
            () -> -gamepad1.left_stick_y,
            () -> -gamepad1.left_stick_x,
            () -> -gamepad1.right_stick_x,
            
            () -> fieldCentric,
            
            () -> slowMode
                ? SLOW_MODE_MULTIPLIER
                : 1.0,
            
            () -> DRIVE_DEADBAND
        )
    );
    
    Scheduler.schedule(
        LiftCommands.manualControl(
            robot.lift,
            () -> -gamepad2.left_stick_y
        )
    );
  }
  
  @Override
  protected void onRobotLoop(double dtSeconds) {
    updateDriveControls();
    updateLiftControls();
  }
  
  private void updateDriveControls() {
    if (gamepad1.leftBumperWasPressed()) {
      slowMode = !slowMode;
    }
    
    if (gamepad1.yWasPressed()) {
      fieldCentric = !fieldCentric;
    }
  }
  
  private void updateLiftControls() {
    /*
     * Отклонение стика отменяет активное движение к пресету.
     * После отмены возобновляется ручная команда.
     */
    if (Math.abs(gamepad2.left_stick_y)
        > LiftConstants.MANUAL_DEADBAND) {
      cancelActiveLiftPositionCommand();
      return;
    }
    
    if (gamepad2.aWasPressed()) {
      scheduleLiftPreset(
          LiftSubsystem.Preset.BOTTOM
      );
    } else if (gamepad2.xWasPressed()) {
      scheduleLiftPreset(
          LiftSubsystem.Preset.MIDDLE
      );
    } else if (gamepad2.yWasPressed()) {
      scheduleLiftPreset(
          LiftSubsystem.Preset.TOP
      );
    }
  }
  
  private void scheduleLiftPreset(
      LiftSubsystem.Preset preset
  ) {
    Command command = LiftCommands.moveToPreset(
        robot.lift,
        preset
    );
    
    activeLiftPositionCommand = command;
    Scheduler.schedule(command);
  }
  
  private void cancelActiveLiftPositionCommand() {
    if (activeLiftPositionCommand == null) {
      return;
    }
    
    if (activeLiftPositionCommand.isScheduled()) {
      activeLiftPositionCommand.cancel();
    }
    
    activeLiftPositionCommand = null;
  }
  
  @Override
  protected void onRobotTelemetry() {
    Pose pose = robot.drive.getPose();
    
    telemetry.addData(
        "Drive mode",
        fieldCentric
            ? "Field-centric"
            : "Robot-centric"
    );
    telemetry.addData("Slow mode", slowMode);
    
    telemetry.addData("X", "%.2f", pose.getX());
    telemetry.addData("Y", "%.2f", pose.getY());
    telemetry.addData(
        "Heading",
        "%.1f deg",
        Math.toDegrees(pose.getHeading())
    );
    
    telemetry.addData(
        "Lift mode",
        robot.lift.getControlMode()
    );
    telemetry.addData(
        "Lift positions",
        "L %d | R %d | avg %.1f",
        robot.lift.getLeftPositionTicks(),
        robot.lift.getRightPositionTicks(),
        robot.lift.getAveragePositionTicks()
    );
    telemetry.addData(
        "Lift target",
        robot.lift.getTargetPositionTicks()
    );
    telemetry.addData(
        "Lift error",
        "%.1f",
        robot.lift.getPositionErrorTicks()
    );
    telemetry.addData(
        "Lift sync error",
        robot.lift.getSynchronizationErrorTicks()
    );
    telemetry.addData(
        "Lift power",
        "%.2f",
        robot.lift.getAppliedPower()
    );
  }
}