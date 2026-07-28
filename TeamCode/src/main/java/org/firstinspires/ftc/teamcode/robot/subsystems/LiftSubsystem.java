package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.core.AbstractSubsystem;

import java.util.Objects;

public final class LiftSubsystem extends AbstractSubsystem {

    private static final DcMotorSimple.Direction LEFT_DIRECTION =
            DcMotorSimple.Direction.FORWARD;

    private static final DcMotorSimple.Direction RIGHT_DIRECTION =
            DcMotorSimple.Direction.REVERSE;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    private double requestedPower;

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
        requestedPower = 0.0;

        configureMotor(leftMotor, LEFT_DIRECTION);
        configureMotor(rightMotor, RIGHT_DIRECTION);
    }

    @Override
    protected void onStart() {
        requestedPower = 0.0;
    }

    @Override
    protected void onPeriodic(double dtSeconds) {
        applyPower(requestedPower);
    }

    @Override
    protected void stopOutputs() {
        requestedPower = 0.0;
        applyPower(0.0);
    }

    public void setManualPower(double power) {
        if (!isEnabled()) {
            return;
        }

        if (Double.isNaN(power) || Double.isInfinite(power)) {
            throw new IllegalArgumentException(
                    "Lift power must be finite"
            );
        }

        requestedPower = Range.clip(power, -1.0, 1.0);
    }

    public void stopMovement() {
        requestedPower = 0.0;
    }

    public double getRequestedPower() {
        return requestedPower;
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

    public int getSynchronizationErrorTicks() {
        return getLeftPositionTicks()
                - getRightPositionTicks();
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
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}