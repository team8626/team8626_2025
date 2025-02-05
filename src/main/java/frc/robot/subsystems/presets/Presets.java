package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;

public class Presets {
  // Will be finished later when we have Algae and Coral Locations Set
  public static final Preset kStart = new Preset("START", "CORAL", 180, 0);
  public static final Preset kStow = new Preset("STOW", "CORAL", 195, 0);
  public static final Preset kCoralL4 = new Preset("CORAL L4", "CORAL", 0, 0, 0, 1275);
  public static final Preset kCoralL1 = new Preset("CORAL L1", "CORAL", 0, 0, 0, 500);
  public static final Preset kSourcePickup = new Preset("SOURCE PICKUP", "CORAL", 127, 11);
  public static final Preset kAReeflgaePickup = new Preset("REEF PICKUP", "ALGAE", 127, 11);
  public static final Preset kReefGroundPickup = new Preset("GROUND PICKUP", "ALGAE", 127, 11);
  public static final Preset kClimbPreset = new Preset("CLIMB PRESET", "EMPTY", 80, 0);
  public static final Preset kClimbEnd = new Preset("CLIMB FINISH", "EMPTY", 198, 0);

  public record Preset(
      String Name,
      String gamePieceType,
      Pose2d robotPose,
      double elevatorHeightInches,
      double WristAngleDegrees,
      double AlgaeRPM,
      double CoralRPM) {

    public Preset(
        String Name, String gamePiece, double elevatorHeightInches, double WristAngleDegrees) {
      this(Name, gamePiece, new Pose2d(), elevatorHeightInches, WristAngleDegrees, 0, 0);
    }

    public Preset(
        String Name,
        String gamePiece,
        double elevatorHeightInches,
        double WristAngleDegrees,
        double AlgaeRPM,
        double CoralRPM) {
      this(
          Name,
          gamePiece,
          new Pose2d(),
          elevatorHeightInches,
          WristAngleDegrees,
          AlgaeRPM,
          CoralRPM);
    }
  }
}
