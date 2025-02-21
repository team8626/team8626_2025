package frc.robot.subsystems.presets;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;

public class AlgaePreset {
  public String name = "'";
  public Pose2d robotPose = new Pose2d();
  public double elevatorHeightInches = 0;
  public double wristAngleDegrees = 0;
  public double RPM = 0;
  public boolean isReady = false;

  /**
   * Create a new Algae Preset
   *
   * @param name
   * @param robotPose
   * @param algaeRPM
   * @param isReady
   */
  public AlgaePreset(
      String name,
      Pose2d robotPose,
      double elevatorHeightInches,
      double wristAngleDegrees,
      double RPM,
      boolean isReady) {
    this.name = name;
    this.robotPose = robotPose;
    this.elevatorHeightInches = elevatorHeightInches;
    this.RPM = RPM;
    this.isReady = isReady;
  }

  public void setElevatorHeightInches(double newHeight) {
    // Set the Elevator Height of the Algae Shooter
    this.elevatorHeightInches = newHeight;
  }

  public void setWristAngleDegrees(double newAngle) {
    // Set the Wrist Angle of the Algae Shooter
    this.wristAngleDegrees = newAngle;
  }

  public void setRPM(double newRPM) {
    // Set the RPM of the Algae Shooter
    this.RPM = newRPM;
  }

  public void setPose(Pose2d newPose) {
    // Set the Pose of the Robot
    this.robotPose = newPose;
  }

  public void setReady(boolean ready) {
    // Set the Ready State of the Preset
    this.isReady = ready;
  }

  public double getRPM() {
    // Get the RPM of the Coral Shooter
    return this.RPM;
  }

  public boolean isReady() {
    // Get the Ready State of the Preset
    return this.isReady;
  }
}
