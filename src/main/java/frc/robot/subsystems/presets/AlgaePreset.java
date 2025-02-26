package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;

public class AlgaePreset {
  private Supplier<Pose2d> robotPoseSupplier;
  private AlgaeLevelPreset subsystemsPresets;
  private boolean isReady = false;

  /**
   * Create a new Algae Preset
   *
   * @param robotPose
   * @param subsystemsPresets
   * @param isReady
   */
  public AlgaePreset(
      String name, Pose2d robotPose, AlgaeLevelPreset subsystemsPresets, boolean isReady) {
    this.robotPoseSupplier = () -> robotPose;
    this.subsystemsPresets = subsystemsPresets;
    this.isReady = isReady;
  }

  public void setPose(Pose2d newPose) {
    // Set the Pose of the Robot
    this.robotPoseSupplier = () -> newPose;
    System.out.printf(
        "Algae Pose Set to - x: %3f, y: %3f, theta: %3f",
        robotPoseSupplier.get().getX(),
        robotPoseSupplier.get().getY(),
        robotPoseSupplier.get().getRotation().getDegrees());
  }

  public void setPosition(AlgaePosition newPosition) {
    // Set the Position of the Algae Shooter
    subsystemsPresets = newPosition.level();
  }

  // public void setPositionByElements(AlgaePosition newPosition) {
  //   // Set the Position of the Algae Shooter
  //   this.elevatorHeightInches = newPosition.level().elevatorHeightInches();
  //   this.wristAngleDegrees = newPosition.level().wristAngleDegrees();
  //   this.RPM = newPosition.level().RPM();
  // }

  public void setReady(boolean ready) {
    // Set the Ready State of the Preset
    this.isReady = ready;
  }

  public double getRPM() {
    // Get the RPM of the Coral Shooter
    return subsystemsPresets.RPM;
  }

  public double getElevatorHeightInches() {
    return subsystemsPresets.elevatorHeightInches();
    // return this.elevatorHeightInches;
  }

  public double getWristAngleDegrees() {
    return subsystemsPresets.wristAngleDegrees();
  }

  public Pose2d getPose() {
    return this.robotPoseSupplier.get();
  }

  public boolean isReady() {
    // Get the Ready State of the Preset
    return this.isReady;
  }

  public enum AlgaeLevelPreset {
    PROCESSOR(8, 190, 1000),
    FLOOR(8, 200, -600),
    LOW(39, 210, -1000),
    HIGH(51, 210, -600);

    AlgaeLevelPreset(double elevatorHeightInches, double wristAngleDegrees, double RPM) {}

    public double elevatorHeightInches() {
      return elevatorHeightInches;
    }

    public double wristAngleDegrees() {
      return wristAngleDegrees;
    }

    public double RPM() {
      return RPM;
    }

    private static double elevatorHeightInches;
    private static double wristAngleDegrees;
    private double RPM;
  }

  public enum AlgaePosition {
    AB(AlgaeLevelPreset.HIGH),
    CD(AlgaeLevelPreset.LOW),
    EF(AlgaeLevelPreset.HIGH),
    GH(AlgaeLevelPreset.LOW),
    IJ(AlgaeLevelPreset.HIGH),
    KL(AlgaeLevelPreset.LOW),
    FLOOR(AlgaeLevelPreset.FLOOR),
    PROCESSOR(AlgaeLevelPreset.PROCESSOR);

    AlgaePosition(AlgaeLevelPreset level) {
      this.level = level;
    }

    public AlgaeLevelPreset level() {
      return this.level;
    }

    private final AlgaeLevelPreset level;
  }
}
