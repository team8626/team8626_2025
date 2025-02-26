package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotConstants;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import java.util.function.Supplier;

public class CoralPreset {
  public String name = "-";
  public Supplier<Pose2d> robotPoseSupplier;
  public double RPMLeft = CoralShooterConstants.RPMShootLeft;
  public double RPMRight = CoralShooterConstants.RPMShootRight;
  public boolean isReady = false;

  // enum ReefBranche{
  //   A, B, C, D, E, F, G, H, I, J, K, L
  // }

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
    this.robotPoseSupplier = () -> robotPose;
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
    this.robotPoseSupplier = () -> newPose;
    System.out.printf(
        "Coral Pose Set to - x: %3f, y: %3f, theta: %3f",
        robotPoseSupplier.get().getX(),
        robotPoseSupplier.get().getY(),
        robotPoseSupplier.get().getRotation().getDegrees());
  }

  public void setLevelPreset(CoralLevelPreset newLevel) {
    // Set the Level of the Coral Shooter
    this.RPMLeft = newLevel.RPMLeft();
    this.RPMRight = newLevel.RPMRight();
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

  public static final YACoralLevelPreset gains =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new YACoralLevelPreset(0.5, 0.0, 0.0, 0.43, 3.07, 0.04);
        case SIMBOT -> new YACoralLevelPreset(0.5, 0.0, 0.0, 0.43, 3.07, 0.04);
        default -> new YACoralLevelPreset(0.05, 0.0, 0.0, 0.43, 3.07, 0.04);
      };

  public record YACoralLevelPreset(
      double kP, double kI, double kD, double kS, double kV, double kA) {}
}

enum CoralLevelPreset {
  L1(500, 500),
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
