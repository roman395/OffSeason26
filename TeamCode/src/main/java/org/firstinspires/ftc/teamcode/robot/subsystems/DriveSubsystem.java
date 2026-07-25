package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.core.AbstractSubsystem;

import java.util.Objects;

public final class DriveSubsystem extends AbstractSubsystem {
  
  private final Follower follower;
  private boolean teleOpActive;
  
  public DriveSubsystem(Follower follower) {
    super("Drive");
    
    this.follower = Objects.requireNonNull(
        follower,
        "follower cannot be null"
    );
  }
  
  @Override
  protected void onInit() {
    follower.setStartingPose(new Pose());
    follower.update();
  }
  
  @Override
  protected void onPeriodic(double dtSeconds) {
    follower.update();
  }
  
  @Override
  protected void stopOutputs() {
    follower.breakFollowing();
    teleOpActive = false;
  }
  
  public void setStartingPose(Pose pose) {
    follower.setStartingPose(
        Objects.requireNonNull(pose, "pose cannot be null")
    );
  }
  
  public void startTeleOp() {
    startTeleOp(true);
  }
  
  public void startTeleOp(boolean useBrakeMode) {
    follower.startTeleOpDrive(useBrakeMode);
    teleOpActive = true;
  }
  
  public void driveRobotCentric(
      double forward,
      double strafe,
      double turn
  ) {
    drive(forward, strafe, turn, true);
  }
  
  public void driveFieldCentric(
      double forward,
      double strafe,
      double turn
  ) {
    drive(forward, strafe, turn, false);
  }
  
  public void stopDriving() {
    drive(0.0, 0.0, 0.0, true);
  }
  
  public boolean isBusy() {
    return follower.isBusy();
  }
  
  public Pose getPose() {
    return follower.getPose();
  }
  
  public Follower getFollower() {
    return follower;
  }
  
  private void drive(
      double forward,
      double strafe,
      double turn,
      boolean robotCentric
  ) {
    if (!teleOpActive || follower.isBusy()) {
      return;
    }
    
    follower.setTeleOpDrive(
        Range.clip(forward, -1.0, 1.0),
        Range.clip(strafe, -1.0, 1.0),
        Range.clip(turn, -1.0, 1.0),
        robotCentric
    );
  }
}