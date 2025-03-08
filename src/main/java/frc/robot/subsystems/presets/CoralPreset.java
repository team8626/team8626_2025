package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotConstants.UIConstants.CoralLevel;

public class CoralPreset {
  public String name = "-";
  public Pose2d robotPose;
  public double RPMLeft = 0;
  public double RPMRight = 0;
  public boolean hasPose = false;
  public boolean hasLevel = false;

  public CoralPreset(String name) {
    this.name = name;
  }

  public CoralPreset(String name, double RPMLeft, double RPMRight) {
    this.name = name;
    this.RPMLeft = RPMLeft;
    this.RPMRight = RPMRight;
    this.hasLevel = true;
    this.hasPose = true;
  }

  public void setPose(Pose2d newPose) {
    // Set the Pose of the Robot
    this.robotPose = newPose;
    this.hasPose = true;
    System.out.printf(
        "[CORALPRESET] New Pose Set to - x: %3f, y: %3f, theta: %3f\n",
        robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees());
  }

  public void setLevel(CoralLevel newLevel) {
    // Set the Level of the Coral Shooter
    switch (newLevel) {
      case L1:
        this.RPMLeft = Presets.CORAL_L1.RPMLeft;
        this.RPMRight = Presets.CORAL_L1.RPMRight;
        this.hasLevel = true;
        break;
      case L2:
        this.RPMLeft = Presets.CORAL_L2.RPMLeft;
        this.RPMRight = Presets.CORAL_L2.RPMRight;
        this.hasLevel = true;
        break;
      case L3:
        this.RPMLeft = Presets.CORAL_L3.RPMLeft;
        this.RPMRight = Presets.CORAL_L3.RPMRight;
        this.hasLevel = true;
        break;
      case L4:
      default:
        this.RPMLeft = Presets.CORAL_L4.RPMLeft;
        this.RPMRight = Presets.CORAL_L4.RPMRight;
        this.hasLevel = true;
        break;
    }
    System.out.printf(
        "[CORALPRESET] New Coral Preset - %s - %.0f/%.0f RPM\n",
        newLevel.toString(), this.RPMLeft, this.RPMRight);
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
    return hasPose ? this.robotPose : null;
  }

  private boolean isReady() {
    // Get the Ready State of the Preset
    return this.hasLevel && this.hasPose;
  }

  public void reset() {
    // Set the Ready State of the Preset
    this.hasPose = false;
  }
}
