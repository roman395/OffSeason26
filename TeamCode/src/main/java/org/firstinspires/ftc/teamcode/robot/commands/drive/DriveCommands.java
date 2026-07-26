package org.firstinspires.ftc.teamcode.robot.commands.drive;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.robot.subsystems.DriveSubsystem;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public final class DriveCommands {
  private static final int AUTOMATIC_DRIVE_PRIORITY = 10;
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
  public static Command followPath(
      DriveSubsystem drive,
      PathChain path
  ) {
    Objects.requireNonNull(path, "path cannot be null");
    
    return configureAutomaticDrive(
        drive,
        PedroCommands.follow(
            drive.getFollower(),
            path
        )
    );
  }
  
  public static Command followPath(
      DriveSubsystem drive,
      PathChain path,
      boolean holdEnd
  ) {
    Objects.requireNonNull(path, "path cannot be null");
    
    return configureAutomaticDrive(
        drive,
        PedroCommands.follow(
            drive.getFollower(),
            path,
            holdEnd
        )
    );
  }
  
  public static Command followPath(
      DriveSubsystem drive,
      PathChain path,
      double maxPower
  ) {
    Objects.requireNonNull(path, "path cannot be null");
    
    return configureAutomaticDrive(
        drive,
        PedroCommands.follow(
            drive.getFollower(),
            path,
            maxPower
        )
    );
  }
  
  public static Command followPath(
      DriveSubsystem drive,
      PathChain path,
      boolean holdEnd,
      double maxPower
  ) {
    Objects.requireNonNull(path, "path cannot be null");
    
    return configureAutomaticDrive(
        drive,
        PedroCommands.follow(
            drive.getFollower(),
            path,
            holdEnd,
            maxPower
        )
    );
  }
  
  public static Command turnTo(
      DriveSubsystem drive,
      double headingRadians
  ) {
    return configureAutomaticDrive(
        drive,
        PedroCommands.turnTo(
            drive.getFollower(),
            headingRadians
        )
    );
  }
  
  public static Command turnTo(
      DriveSubsystem drive,
      double headingRadians,
      PathConstraints constraints
  ) {
    Objects.requireNonNull(
        constraints,
        "constraints cannot be null"
    );
    
    return configureAutomaticDrive(
        drive,
        PedroCommands.turnTo(
            drive.getFollower(),
            headingRadians,
            constraints
        )
    );
  }
  
  public static Command holdCurrentPose(
      DriveSubsystem drive
  ) {
    return configureAutomaticDrive(
        drive,
        PedroCommands.hold(
            drive.getFollower()
        )
    );
  }
  
  public static Command holdPose(
      DriveSubsystem drive,
      Pose pose
  ) {
    Objects.requireNonNull(pose, "pose cannot be null");
    
    return configureAutomaticDrive(
        drive,
        PedroCommands.hold(
            drive.getFollower(),
            pose
        )
    );
  }
  
  public static Command holdPose(
      DriveSubsystem drive,
      Pose pose,
      PathConstraints constraints
  ) {
    Objects.requireNonNull(pose, "pose cannot be null");
    Objects.requireNonNull(
        constraints,
        "constraints cannot be null"
    );
    
    return configureAutomaticDrive(
        drive,
        PedroCommands.hold(
            drive.getFollower(),
            pose,
            constraints
        )
    );
  }
  
  private static Command configureAutomaticDrive(
      DriveSubsystem drive,
      CommandBuilder command
  ) {
    Objects.requireNonNull(drive, "drive cannot be null");
    Objects.requireNonNull(command, "command cannot be null");
    
    return command
        .requiring(drive)
        .setPriority(AUTOMATIC_DRIVE_PRIORITY)
        .setEnd(endCondition -> {
          if (endCondition == EndCondition.NATURALLY) {
            drive.completeAutomaticDrive();
          } else {
            drive.cancelAutomaticDrive();
          }
        });
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