package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.config.LiftConstants;
import org.firstinspires.ftc.teamcode.robot.core.AbstractSubsystem;

import java.util.Objects;

public final class LiftSubsystem extends AbstractSubsystem {
  
  public enum ControlMode {
    MANUAL,
    POSITION
  }
  
  public enum Preset {
    BOTTOM(LiftConstants.BOTTOM_TICKS),
    MIDDLE(LiftConstants.MIDDLE_TICKS),
    TOP(LiftConstants.TOP_TICKS);
    
    private final int positionTicks;
    
    Preset(int positionTicks) {
      this.positionTicks = positionTicks;
    }
    
    public int getPositionTicks() {
      return positionTicks;
    }
  }
  
  private static final DcMotorSimple.Direction LEFT_DIRECTION =
      DcMotorSimple.Direction.FORWARD;
  
  private static final DcMotorSimple.Direction RIGHT_DIRECTION =
      DcMotorSimple.Direction.REVERSE;
  
  private final DcMotorEx leftMotor;
  private final DcMotorEx rightMotor;
  
  private ControlMode controlMode = ControlMode.MANUAL;
  
  private double manualPower;
  private double appliedPower;
  
  private int targetPositionTicks;
  private double positionPowerLimit;
  
  private double kP = LiftConstants.KP;
  private double kI = LiftConstants.KI;
  private double kD = LiftConstants.KD;
  
  private double integralError;
  private double previousError;
  private boolean hasPreviousError;
  
  public LiftSubsystem(
      DcMotorEx leftMotor,
      DcMotorEx rightMotor
  ) {
    super("Lift");
    
    this.leftMotor = Objects.requireNonNull(
        leftMotor,
        "leftMotor cannot be null"
    );
    
    this.rightMotor = Objects.requireNonNull(
        rightMotor,
        "rightMotor cannot be null"
    );
  }
  
  @Override
  protected void onInit() {
    configureMotor(leftMotor, LEFT_DIRECTION);
    configureMotor(rightMotor, RIGHT_DIRECTION);
    
    controlMode = ControlMode.MANUAL;
    manualPower = 0.0;
    appliedPower = 0.0;
    
    targetPositionTicks = 0;
    positionPowerLimit = 0.0;
    
    resetPidState();
  }
  
  @Override
  protected void onStart() {
    controlMode = ControlMode.MANUAL;
    manualPower = 0.0;
    targetPositionTicks = (int) Math.round(
        getAveragePositionTicks()
    );
    resetPidState();
  }
  
  @Override
  protected void onPeriodic(double dtSeconds) {
    if (controlMode == ControlMode.POSITION) {
      applyPower(calculatePositionPower(dtSeconds));
      return;
    }
    
    applyPower(manualPower);
  }
  
  @Override
  protected void stopOutputs() {
    controlMode = ControlMode.MANUAL;
    manualPower = 0.0;
    positionPowerLimit = 0.0;
    resetPidState();
    applyPower(0.0);
  }
  
  public void setManualPower(double power) {
    if (!isEnabled()) {
      return;
    }
    
    requireFinite(power, "Lift power");
    
    if (controlMode != ControlMode.MANUAL) {
      controlMode = ControlMode.MANUAL;
      resetPidState();
    }
    
    manualPower = Range.clip(power, -1.0, 1.0);
  }
  
  public void moveToPosition(
      int targetPositionTicks,
      double maxPower
  ) {
    if (!isEnabled()) {
      return;
    }
    
    requireFinite(maxPower, "Position power limit");
    
    if (maxPower <= 0.0 || maxPower > 1.0) {
      throw new IllegalArgumentException(
          "Position power limit must be in (0.0, 1.0]"
      );
    }
    
    boolean targetChanged =
        this.targetPositionTicks != targetPositionTicks;
    
    if (controlMode != ControlMode.POSITION || targetChanged) {
      resetPidState();
    }
    
    this.targetPositionTicks = targetPositionTicks;
    positionPowerLimit = maxPower;
    controlMode = ControlMode.POSITION;
  }
  
  public void moveToPosition(int targetPositionTicks) {
    moveToPosition(
        targetPositionTicks,
        LiftConstants.POSITION_MAX_POWER
    );
  }
  
  public void moveToPreset(Preset preset) {
    Objects.requireNonNull(preset, "preset");
    moveToPosition(preset.getPositionTicks());
  }
  
  public void holdCurrentPosition(double maxPower) {
    moveToPosition(
        (int) Math.round(getAveragePositionTicks()),
        maxPower
    );
  }
  
  public void holdCurrentPosition() {
    holdCurrentPosition(
        LiftConstants.POSITION_MAX_POWER
    );
  }
  
  public void stopMovement() {
    controlMode = ControlMode.MANUAL;
    manualPower = 0.0;
    positionPowerLimit = 0.0;
    resetPidState();
    applyPower(0.0);
  }
  
  public void setPidCoefficients(
      double kP,
      double kI,
      double kD
  ) {
    requireFinite(kP, "kP");
    requireFinite(kI, "kI");
    requireFinite(kD, "kD");
    
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    resetPidState();
  }
  
  public ControlMode getControlMode() {
    return controlMode;
  }
  
  public double getManualPower() {
    return manualPower;
  }
  
  public double getAppliedPower() {
    return appliedPower;
  }
  
  public int getTargetPositionTicks() {
    return targetPositionTicks;
  }
  
  public double getPositionPowerLimit() {
    return positionPowerLimit;
  }
  
  public int getLeftPositionTicks() {
    return leftMotor.getCurrentPosition();
  }
  
  public int getRightPositionTicks() {
    return rightMotor.getCurrentPosition();
  }
  
  public double getAveragePositionTicks() {
    return (
        getLeftPositionTicks()
            + getRightPositionTicks()
    ) / 2.0;
  }
  
  public double getPositionErrorTicks() {
    return targetPositionTicks
        - getAveragePositionTicks();
  }
  
  public int getSynchronizationErrorTicks() {
    return getLeftPositionTicks()
        - getRightPositionTicks();
  }
  
  public boolean isAtTarget(int toleranceTicks) {
    if (toleranceTicks < 0) {
      throw new IllegalArgumentException(
          "Position tolerance cannot be negative"
      );
    }
    
    return controlMode == ControlMode.POSITION
        && Math.abs(getPositionErrorTicks()) <= toleranceTicks;
  }
  
  public boolean isAtTarget() {
    return isAtTarget(
        LiftConstants.POSITION_TOLERANCE_TICKS
    );
  }
  
  private double calculatePositionPower(double dtSeconds) {
    double error = getPositionErrorTicks();
    double derivative = 0.0;
    
    if (hasPreviousError && dtSeconds > 0.0) {
      derivative = (error - previousError) / dtSeconds;
    }
    
    if (kI == 0.0) {
      integralError = 0.0;
    } else if (dtSeconds > 0.0) {
      double maxIntegralError =
          positionPowerLimit / Math.abs(kI);
      
      integralError = Range.clip(
          integralError + error * dtSeconds,
          -maxIntegralError,
          maxIntegralError
      );
    }
    
    previousError = error;
    hasPreviousError = true;
    
    double output =
        kP * error
            + kI * integralError
            + kD * derivative;
    
    return Range.clip(
        output,
        -positionPowerLimit,
        positionPowerLimit
    );
  }
  
  private void resetPidState() {
    integralError = 0.0;
    previousError = 0.0;
    hasPreviousError = false;
  }
  
  private void configureMotor(
      DcMotorEx motor,
      DcMotorSimple.Direction direction
  ) {
    motor.setPower(0.0);
    motor.setDirection(direction);
    
    motor.setZeroPowerBehavior(
        DcMotor.ZeroPowerBehavior.BRAKE
    );
    
    motor.setMode(
        DcMotor.RunMode.STOP_AND_RESET_ENCODER
    );
    
    motor.setMode(
        DcMotor.RunMode.RUN_WITHOUT_ENCODER
    );
  }
  
  private void applyPower(double power) {
    appliedPower = power;
    leftMotor.setPower(power);
    rightMotor.setPower(power);
  }
  
  private static void requireFinite(
      double value,
      String name
  ) {
    if (Double.isNaN(value) || Double.isInfinite(value)) {
      throw new IllegalArgumentException(
          name + " must be finite"
      );
    }
  }
  
}