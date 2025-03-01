package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import java.util.function.Supplier;

public class CoralPreset {
  public String name = "-";
  public Supplier<Pose2d> robotPoseSupplier;
  public double RPMLeft = CoralShooterConstants.RPMShootLeft;
  public double RPMRight = CoralShooterConstants.RPMShootRight;
  public boolean isReady = false;

  public CoralPreset(String name) {
    this.name = name;
    this.isReady = false;
  }

  public CoralPreset(
      String name, Pose2d robotPose, double RPMLeft, double RPMRight, boolean isReady) {
    this.name = name;
    this.robotPoseSupplier = () -> robotPose;
    this.RPMLeft = RPMLeft;
    this.RPMRight = RPMRight;
    this.isReady = isReady;
  }

  public CoralPreset(String name, double RPMLeft, double RPMRight) {
    this.name = name;
    this.RPMLeft = RPMLeft;
    this.RPMRight = RPMRight;
    this.isReady = true;
  }

  public void setPose(Pose2d newPose) {
    // Set the Pose of the Robot
    this.robotPoseSupplier = () -> newPose;
    System.out.printf(
        "[CORAL PRESET] New Pose Set to - x: %3f, y: %3f, theta: %3f",
        robotPoseSupplier.get().getX(),
        robotPoseSupplier.get().getY(),
        robotPoseSupplier.get().getRotation().getDegrees());
  }

  public void setSubsystems(CoralLevelPreset newLevel) {
    // Set the Level of the Coral Shooter
    switch (newLevel) {
      case L1:
        this.RPMLeft = Presets.CORAL_L1.RPMLeft;
        this.RPMRight = Presets.CORAL_L1.RPMRight;
        break;
      case L2:
        this.RPMLeft = Presets.CORAL_L2.RPMLeft;
        this.RPMRight = Presets.CORAL_L2.RPMRight;
        break;
      case L3:
        this.RPMLeft = Presets.CORAL_L3.RPMLeft;
        this.RPMRight = Presets.CORAL_L3.RPMRight;
        break;
      case L4:
      default:
        this.RPMLeft = Presets.CORAL_L4.RPMLeft;
        this.RPMRight = Presets.CORAL_L4.RPMRight;
        break;
    }
    System.out.printf(
        "[CORALPRESET] New Coral Preset - %s - %.0f/%.0f RPM\n",
        newLevel.toString(), this.RPMLeft, this.RPMRight);
  }

  public void setReady(boolean ready) {
    // Set the Ready State of the Preset
    this.isReady = ready;
  }

  public double getRPMLeft() {
    // Get the RPM of the Coral Shooter
    return this.RPMLeft;
  }

  public double getRPMRight() {
    // Get the RPM of the Coral Shooter
    return this.RPMRight;
  }

  public Pose2d getPose() {
    // Get the Pose of the Coral Shooter
    return this.robotPoseSupplier.get();
  }

  public boolean isReady() {
    // Get the Ready State of the Preset
    return this.isReady;
  }
}

enum CoralLevelPreset {
  L1(900, 1000),
  L2(500, 500),
  L3(500, 500),
  L4(1275, 1275);

  CoralLevelPreset(int RMPLeft, int RMPRight) {}

  public double RPMLeft() {
    return RPMLeft;
  }

  public double RPMRight() {
    return RPMRight;
  }

  private static double RPMLeft;
  private static double RPMRight;
}
