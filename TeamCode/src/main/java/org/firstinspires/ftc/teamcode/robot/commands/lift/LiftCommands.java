package org.firstinspires.ftc.teamcode.robot.commands.lift;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.config.LiftConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.LiftSubsystem;

import java.util.Objects;
import java.util.function.DoubleSupplier;

public final class LiftCommands {
  
  private static final int POSITION_PRIORITY = 10;
  
  private LiftCommands() {
    // Utility class: instances are not needed.
  }
  
  public static Command manualControl(
      LiftSubsystem lift,
      DoubleSupplier powerSupplier
  ) {
    Objects.requireNonNull(
        lift,
        "lift cannot be null"
    );
    
    Objects.requireNonNull(
        powerSupplier,
        "powerSupplier cannot be null"
    );
    
    return Command.build()
        .setExecute(() -> {
          double input = powerSupplier.getAsDouble();
          
          if (Double.isNaN(input)
              || Double.isInfinite(input)) {
            throw new IllegalArgumentException(
                "Lift input must be finite"
            );
          }
          
          input = Range.clip(input, -1.0, 1.0);
          
          if (Math.abs(input)
              <= LiftConstants.MANUAL_DEADBAND) {
            input = 0.0;
          }
          
          lift.setManualPower(
              input * LiftConstants.MANUAL_MAX_POWER
          );
        })
        
        .setEnd(endCondition ->
            lift.stopMovement()
        )
        
        .requiring(lift)
        
        .setInterruptedBehavior(
            InterruptedBehavior.SUSPEND
        );
  }
  
  public static Command moveToPreset(
      LiftSubsystem lift,
      LiftSubsystem.Preset preset
  ) {
    Objects.requireNonNull(
        preset,
        "preset cannot be null"
    );
    
    return positionCommand(
        lift,
        () -> lift.moveToPreset(preset)
    );
  }
  
  public static Command moveToPosition(
      LiftSubsystem lift,
      int targetPositionTicks
  ) {
    return positionCommand(
        lift,
        () -> lift.moveToPosition(targetPositionTicks)
    );
  }
  
  public static Command moveToPosition(
      LiftSubsystem lift,
      int targetPositionTicks,
      double maxPower
  ) {
    return positionCommand(
        lift,
        () -> lift.moveToPosition(
            targetPositionTicks,
            maxPower
        )
    );
  }
  
  private static Command positionCommand(
      LiftSubsystem lift,
      Runnable startAction
  ) {
    Objects.requireNonNull(
        lift,
        "lift cannot be null"
    );
    
    Objects.requireNonNull(
        startAction,
        "startAction cannot be null"
    );
    
    return Command.build()
        .setStart(startAction)
        
        .setDone(lift::isAtTarget)
        
        .setEnd(endCondition -> {
          if (endCondition
              != EndCondition.NATURALLY) {
            lift.stopMovement();
          }
        })
        
        .requiring(lift)
        
        .setPriority(POSITION_PRIORITY);
  }
  
}