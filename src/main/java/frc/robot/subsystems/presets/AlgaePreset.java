package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;

public class AlgaePreset {
  private Supplier<Pose2d> robotPoseSupplier;
  // private AlgaeLevelPreset subsystemsPresets;
  private double elevatorHeightInches;
  private double wristAngleDegrees;
  private double RPM;
  private boolean isReady = false;
  private String name;

  public AlgaePreset(String name) {
    this.name = name;
    this.isReady = false;
  }

  public AlgaePreset(
      String name, double elevatorHeightInches, double wristAngleDegrees, double RPM) {
    this.name = name;
    this.elevatorHeightInches = elevatorHeightInches;
    this.wristAngleDegrees = wristAngleDegrees;
    this.RPM = RPM;

    this.isReady = true;
  }

  public void setPose(Pose2d newPose) {
    // Set the Pose of the Robot
    this.robotPoseSupplier = () -> newPose;
    System.out.printf(
        "[ALGAE PRESET] New Pose Set to - x: %3f, y: %3f, theta: %3f",
        robotPoseSupplier.get().getX(),
        robotPoseSupplier.get().getY(),
        robotPoseSupplier.get().getRotation().getDegrees());
  }

  public void setSubsystems(String newPositionName) {
    switch (newPositionName) {
      case "AB":
      case "EF":
      case "IJ":
        this.elevatorHeightInches = Presets.ALGAE_HIGH.getElevatorHeightInches();
        this.wristAngleDegrees = Presets.ALGAE_HIGH.getWristAngleDegrees();
        this.RPM = Presets.ALGAE_HIGH.getRPM();
        break;
      case "CD":
      case "GH":
      case "KL":
        this.elevatorHeightInches = Presets.ALGAE_LOW.getElevatorHeightInches();
        this.wristAngleDegrees = Presets.ALGAE_LOW.getWristAngleDegrees();
        this.RPM = Presets.ALGAE_LOW.getRPM();
        break;
      case "PROCESSOR":
        this.elevatorHeightInches = 8;
        this.wristAngleDegrees = 190;
        this.RPM = 1000;
        break;
      case "FLOOR":
      default:
        this.elevatorHeightInches = 8;
        this.wristAngleDegrees = 200;
        this.RPM = -600;
        break;
    }

    System.out.printf(
        "[ALGAE] New Algae Preset - %s - %s In, %s Deg, %s RPM\n",
        this.toString(), elevatorHeightInches, wristAngleDegrees, RPM);
  }

  public void setReady(boolean ready) {
    // Set the Ready State of the Preset
    this.isReady = ready;
  }

  public double getRPM() {
    // Get the RPM of the Coral Shooter
    return this.RPM;
  }

  public double getElevatorHeightInches() {
    return this.elevatorHeightInches;
    // return this.elevatorHeightInches;
  }

  public double getWristAngleDegrees() {
    return this.wristAngleDegrees;
  }

  public Pose2d getPose() {
    return this.robotPoseSupplier.get();
  }

  public boolean isReady() {
    // Get the Ready State of the Preset
    return this.isReady;
  }
}
