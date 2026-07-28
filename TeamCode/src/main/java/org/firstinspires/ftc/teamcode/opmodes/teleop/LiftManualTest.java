package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmodes.base.BaseRobotOpMode;

@Configurable
@TeleOp(name = "Lift Manual Test", group = "Test")
public final class LiftManualTest extends BaseRobotOpMode {
  
  /*
   * Даже если TEST_POWER_LIMIT случайно выставят выше,
   * диагностический OpMode не даст больше 20% мощности.
   */
  private static final double ABSOLUTE_POWER_LIMIT = 0.20;
  
  public static double TEST_POWER_LIMIT = 0.15;
  public static double DEADBAND = 0.05;
  
  @Override
  protected void onRobotLoop(double dtSeconds) {
    double stickInput = -gamepad2.left_stick_y;
    double deadband = Range.clip(DEADBAND, 0.0, 1.0);
    double powerLimit = Range.clip(
        Math.abs(TEST_POWER_LIMIT),
        0.0,
        ABSOLUTE_POWER_LIMIT
    );
    
    double power = 0.0;
    
    if (gamepad2.left_bumper
        && Math.abs(stickInput) > deadband) {
      power = Range.clip(stickInput, -1.0, 1.0)
          * powerLimit;
    }
    
    robot.lift.setManualPower(power);
  }
  
  @Override
  protected void onRobotTelemetry() {
    telemetry.addLine(
        "Hold gamepad 2 LB and move the left stick"
    );
    telemetry.addLine(
        "Release LB to stop the lift"
    );
    
    telemetry.addData(
        "Safety switch",
        gamepad2.left_bumper ? "ARMED" : "RELEASED"
    );
    telemetry.addData(
        "Power",
        "%.2f",
        robot.lift.getManualPower()
    );
    telemetry.addData(
        "Left ticks",
        robot.lift.getLeftPositionTicks()
    );
    telemetry.addData(
        "Right ticks",
        robot.lift.getRightPositionTicks()
    );
    telemetry.addData(
        "Average ticks",
        "%.1f",
        robot.lift.getAveragePositionTicks()
    );
    telemetry.addData(
        "Synchronization error",
        robot.lift.getSynchronizationErrorTicks()
    );
  }
  
}