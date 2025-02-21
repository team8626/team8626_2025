package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;

public class CoralPreset {
  public String name = "'";
  public Pose2d robotPose = new Pose2d();
  public double RPMLeft = CoralShooterConstants.RPMShootLeft;
  public double RPMRight = CoralShooterConstants.RPMShootRight;
  public boolean isReady = false;

  /**
   * Create a new Coral Preset
   *
   * @param name
   * @param robotPose
   * @param coralRPM
   * @param isReady
   */
  public CoralPreset(
      String name, Pose2d robotPose, double RPMLeft, double RPMRight, boolean isReady) {
    this.name = name;
    this.robotPose = robotPose;
    this.RPMLeft = RPMLeft;
    this.RPMRight = RPMRight;
    this.isReady = isReady;
  }

  public void setRPM(double newRPM) {
    // Set the RPM of the Coral Shooter
    this.RPMLeft = newRPM;
    this.RPMRight = newRPM;
  }

  public void setRPMLeft(double newRPM) {
    // Set the Left RPM of the Coral Shooter
    this.RPMLeft = newRPM;
  }

  public void setRPMRight(double newRPM) {
    // Set the Right RPM of the Coral Shooter
    this.RPMRight = newRPM;
  }

  public void setPose(Pose2d newPose) {
    // Set the Pose of the Robot
    this.robotPose = newPose;
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

  public boolean isReady() {
    // Get the Ready State of the Preset
    return this.isReady;
  }
}
