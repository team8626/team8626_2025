package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;

public class Presets {
  // Will be finished later when we have Algae and Coral Locations Set
  public static final Preset kStart = new Preset("START", 180, 0);
  public static final Preset kStow = new Preset("STOW", 195, 0);
  public static final Preset kCoralL4 = new Preset("CORAL L4", 0, 0, 0, 1275);
  public static final Preset kCoralL1 = new Preset("CORAL L1", 0, 0, 0, 500);
  public static final Preset kSourcePickup = new Preset("SOURCE PICKUP", 127, 11);
  public static final Preset kAReeflgaePickup = new Preset("REEF PICKUP", 127, 11);
  public static final Preset kReefGroundPickup = new Preset("GROUND PICKUP", 127, 11);
  public static final Preset kClimbPreset = new Preset("CLIMB PRESET", 80, 0);
  public static final Preset kClimbEnd = new Preset("CLIMB FINISH", 198, 0);

  public record Preset(
      String Name,
      Pose2d robotPose,
      double elevatorHeightInches,
      double WristAngleDegrees,
      double AlgaeRPM,
      double CoralRPM) {

    public Preset(String Name, double elevatorHeightInches, double WristAngleDegrees) {
      this(Name, new Pose2d(), elevatorHeightInches, WristAngleDegrees, 0, 0);
    }

    public Preset(
        String Name,
        double elevatorHeightInches,
        double WristAngleDegrees,
        double AlgaeRPM,
        double CoralRPM) {
      this(Name, new Pose2d(), elevatorHeightInches, WristAngleDegrees, AlgaeRPM, CoralRPM);
    }
  }
}
