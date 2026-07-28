package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.base.BaseRobotOpMode;
import org.firstinspires.ftc.teamcode.robot.commands.drive.DriveCommands;

@Configurable
@TeleOp(name = "Drive Test", group = "Test")
public final class DriveTestTeleOp extends BaseRobotOpMode {
  
  public static double DEADBAND = 0.05;
  public static double SLOW_MODE_MULTIPLIER = 0.35;
  
  private boolean slowMode;
  private boolean fieldCentric;
  
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
            
            () -> DEADBAND
        )
    );
  }
  
  @Override
  protected void onRobotLoop(double dtSeconds) {
    updateModes();
    updateTelemetry(dtSeconds);
  }
  
  private void updateModes() {
    if (gamepad1.leftBumperWasPressed()) {
      slowMode = !slowMode;
    }
    
    if (gamepad1.yWasPressed()) {
      fieldCentric = !fieldCentric;
    }
  }
  
  private void updateTelemetry(double dtSeconds) {
    Pose pose = robot.drive.getPose();
    
    telemetry.addData(
        "Drive mode",
        fieldCentric ? "Field-centric" : "Robot-centric"
    );
    telemetry.addData("Slow mode", slowMode);
    telemetry.addData("Loop dt", "%.3f s", dtSeconds);
    
    telemetry.addData("X", "%.2f", pose.getX());
    telemetry.addData("Y", "%.2f", pose.getY());
    telemetry.addData(
        "Heading",
        "%.1f deg",
        Math.toDegrees(pose.getHeading())
    );
    
    telemetry.update();
  }
  
}