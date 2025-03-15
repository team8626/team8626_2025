package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotConstants.UIConstants.AlgaeFace2;

public class AlgaePreset {
  private Pose2d robotPose = null;
  private double elevatorHeightInches;
  private double wristAngleDegrees;
  private double RPM;
  public boolean hasPose = false;
  public boolean hasSubsystemsReady = false;
  private String name;

  public AlgaePreset(String name) {
    this.name = name;
    this.hasPose = false;
    this.hasSubsystemsReady = false;

    // TODO: Set Default subsystems to floor
  }

  public AlgaePreset(
      String name, Pose2d pose, double elevatorHeightInches, double wristAngleDegrees, double RPM) {
    this.name = name;
    this.robotPose = pose;
    this.elevatorHeightInches = elevatorHeightInches;
    this.wristAngleDegrees = wristAngleDegrees;
    this.RPM = RPM;
    this.hasSubsystemsReady = true;
    this.hasPose = true;
  }

  public AlgaePreset(
      String name, double elevatorHeightInches, double wristAngleDegrees, double RPM) {
    this.name = name;
    this.elevatorHeightInches = elevatorHeightInches;
    this.wristAngleDegrees = wristAngleDegrees;
    this.RPM = RPM;

    this.hasSubsystemsReady = true;
    this.hasPose = false;
  }

  public void setPose(Pose2d newPose) {
    // Set the Pose of the Robot
    this.robotPose = newPose;
    this.hasPose = true;
    System.out.printf(
        "[ALGAE PRESET] New Pose Set to - x: %3f, y: %3f, theta: %3f",
        robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees());
  }

  public void setSubsystems(AlgaeFace2 newPosition) {
    switch (newPosition) {
      case AB:
      case EF:
      case IJ:
        this.elevatorHeightInches = Presets.ALGAE_HIGH.getElevatorHeightInches();
        this.wristAngleDegrees = Presets.ALGAE_HIGH.getWristAngleDegrees();
        this.RPM = Presets.ALGAE_HIGH.getRPM();
        this.hasSubsystemsReady = true;
        break;
      case CD:
      case GH:
      case KL:
        this.elevatorHeightInches = Presets.ALGAE_LOW.getElevatorHeightInches();
        this.wristAngleDegrees = Presets.ALGAE_LOW.getWristAngleDegrees();
        this.RPM = Presets.ALGAE_LOW.getRPM();
        this.hasSubsystemsReady = true;
        break;
      case FLOOR:
      default:
        this.elevatorHeightInches = Presets.ALGAE_FLOOR.getElevatorHeightInches();
        this.wristAngleDegrees = Presets.ALGAE_FLOOR.getWristAngleDegrees();
        this.RPM = Presets.ALGAE_FLOOR.getRPM();
        break;
    }

    System.out.printf(
        "[ALGAE] New Algae Preset - %s - %s In, %s Deg, %s RPM\n",
        this.toString(), elevatorHeightInches, wristAngleDegrees, RPM);
  }

  private void setSubsystems(String newPositionName) {
    switch (newPositionName) {
      case "AB":
      case "EF":
      case "IJ":
        this.elevatorHeightInches = Presets.ALGAE_HIGH.getElevatorHeightInches();
        this.wristAngleDegrees = Presets.ALGAE_HIGH.getWristAngleDegrees();
        this.RPM = Presets.ALGAE_HIGH.getRPM();
        this.hasSubsystemsReady = true;
        break;
      case "CD":
      case "GH":
      case "KL":
        this.elevatorHeightInches = Presets.ALGAE_LOW.getElevatorHeightInches();
        this.wristAngleDegrees = Presets.ALGAE_LOW.getWristAngleDegrees();
        this.RPM = Presets.ALGAE_LOW.getRPM();
        this.hasSubsystemsReady = true;
        break;
      case "PROCESSOR":
        this.elevatorHeightInches = Presets.ALGAE_PROCESS.getElevatorHeightInches();
        this.wristAngleDegrees = Presets.ALGAE_PROCESS.getWristAngleDegrees();
        this.RPM = Presets.ALGAE_PROCESS.getRPM();
        this.hasSubsystemsReady = true;
        break;
      case "FLOOR":
      default:
        this.elevatorHeightInches = Presets.ALGAE_FLOOR.getElevatorHeightInches();
        this.wristAngleDegrees = Presets.ALGAE_FLOOR.getWristAngleDegrees();
        this.RPM = Presets.ALGAE_FLOOR.getRPM();
        break;
    }

    System.out.printf(
        "[ALGAE] New Algae Preset - %s - %s In, %s Deg, %s RPM\n",
        this.toString(), elevatorHeightInches, wristAngleDegrees, RPM);
  }

  public double getRPM() {
    // Get the RPM of the Coral Shooter
    return this.RPM;
  }

  public void setRPM(double newRPM) {
    this.RPM = newRPM;
  }

  public void setElevatorHeightInches(double newElevatorHeightInches) {
    this.elevatorHeightInches = newElevatorHeightInches;
  }

  public double getElevatorHeightInches() {
    return this.elevatorHeightInches;
  }

  public void setWristAngleDegrees(double newWristAngleDegrees) {
    this.wristAngleDegrees = newWristAngleDegrees;
  }

  public double getWristAngleDegrees() {
    return this.wristAngleDegrees;
  }

  /**
   * Get the Pose of the Robot
   *
   * @return <null>> if the Pose is not set, otherwise the Pose of the Robot
   */
  public Pose2d getPose() {
    return hasPose ? this.robotPose : null;
  }

  private boolean isReady() {
    // Get the Ready State of the Preset
    return this.hasSubsystemsReady && this.hasPose;
  }

  public void reset() {
    this.hasPose = false;

    // Reset to NONE (Floor)
    this.setSubsystems(AlgaeFace2.FLOOR);
  }
}
