package org.firstinspires.ftc.teamcode.robot.commands.drive;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public final class DriveCommands {
  
  private DriveCommands() {
    // Utility class: instances are not needed.
  }
  
  public static Command teleOpDrive(
      DriveSubsystem drive,
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier turnSupplier,
      BooleanSupplier fieldCentricSupplier,
      DoubleSupplier speedMultiplierSupplier,
      DoubleSupplier deadbandSupplier
  ) {
    Objects.requireNonNull(drive, "drive cannot be null");
    Objects.requireNonNull(
        forwardSupplier,
        "forwardSupplier cannot be null"
    );
    Objects.requireNonNull(
        strafeSupplier,
        "strafeSupplier cannot be null"
    );
    Objects.requireNonNull(
        turnSupplier,
        "turnSupplier cannot be null"
    );
    Objects.requireNonNull(
        fieldCentricSupplier,
        "fieldCentricSupplier cannot be null"
    );
    Objects.requireNonNull(
        speedMultiplierSupplier,
        "speedMultiplierSupplier cannot be null"
    );
    Objects.requireNonNull(
        deadbandSupplier,
        "deadbandSupplier cannot be null"
    );
    
    return Command.build()
        .setStart(drive::startTeleOp)
        
        .setExecute(() -> {
          double deadband = Range.clip(
              deadbandSupplier.getAsDouble(),
              0.0,
              0.95
          );
          
          double speedMultiplier = Range.clip(
              speedMultiplierSupplier.getAsDouble(),
              0.0,
              1.0
          );
          
          double forward = shapeInput(
              forwardSupplier.getAsDouble(),
              deadband
          ) * speedMultiplier;
          
          double strafe = shapeInput(
              strafeSupplier.getAsDouble(),
              deadband
          ) * speedMultiplier;
          
          double turn = shapeInput(
              turnSupplier.getAsDouble(),
              deadband
          ) * speedMultiplier;
          
          if (fieldCentricSupplier.getAsBoolean()) {
            drive.driveFieldCentric(
                forward,
                strafe,
                turn
            );
          } else {
            drive.driveRobotCentric(
                forward,
                strafe,
                turn
            );
          }
        })
        
        .setEnd(endCondition -> drive.stopDriving())
        
        .requiring(drive)
        
        .setInterruptedBehavior(
            InterruptedBehavior.SUSPEND
        );
  }
  
  private static double shapeInput(
      double input,
      double deadband
  ) {
    double clippedInput = Range.clip(input, -1.0, 1.0);
    double magnitude = Math.abs(clippedInput);
    
    if (magnitude <= deadband) {
      return 0.0;
    }
    
    double scaledMagnitude =
        (magnitude - deadband) / (1.0 - deadband);
    
    return Math.copySign(scaledMagnitude, clippedInput);
  }
}