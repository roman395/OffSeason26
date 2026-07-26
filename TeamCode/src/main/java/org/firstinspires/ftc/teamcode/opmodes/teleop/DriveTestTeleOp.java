package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmodes.base.BaseRobotOpMode;

@Configurable
@TeleOp(name = "Drive Test", group = "Test")
public final class DriveTestTeleOp extends BaseRobotOpMode {
  
  public static double DEADBAND = 0.05;
  public static double SLOW_MODE_MULTIPLIER = 0.35;
  
  private boolean slowMode;
  private boolean fieldCentric;
  
  @Override
  protected void onRobotStart() {
    robot.drive.startTeleOp();
  }
  
  @Override
  protected void onRobotLoop(double dtSeconds) {
    updateModes();
    
    double speedMultiplier = slowMode
        ? Range.clip(SLOW_MODE_MULTIPLIER, 0.0, 1.0)
        : 1.0;
    
    double forward = applyDeadband(-gamepad1.left_stick_y)
        * speedMultiplier;
    
    double strafe = applyDeadband(-gamepad1.left_stick_x)
        * speedMultiplier;
    
    double turn = applyDeadband(-gamepad1.right_stick_x)
        * speedMultiplier;
    
    if (fieldCentric) {
      robot.drive.driveFieldCentric(forward, strafe, turn);
    } else {
      robot.drive.driveRobotCentric(forward, strafe, turn);
    }
    
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
  
  private static double applyDeadband(double value) {
    double deadband = Range.clip(DEADBAND, 0.0, 0.95);
    double magnitude = Math.abs(value);
    
    if (magnitude <= deadband) {
      return 0.0;
    }
    
    double scaledMagnitude =
        (magnitude - deadband) / (1.0 - deadband);
    
    return Math.copySign(scaledMagnitude, value);
  }
}