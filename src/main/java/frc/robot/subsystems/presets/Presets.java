package frc.robot.subsystems.presets;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Presets {

  public static final AlgaePreset ALGAE_LOW =
      new AlgaePreset("LOW", Inches.of(39), Degrees.of(200), RPM.of(-1000.0));
  public static final AlgaePreset ALGAE_HIGH =
      new AlgaePreset("HIGH", Inches.of(51), Degrees.of(190), RPM.of(-1000));
  public static final AlgaePreset ALGAE_FLOOR =
      new AlgaePreset("FLOOR", Inches.of(11), Degrees.of(195), RPM.of(-1000));
  public static final AlgaePreset ALGAE_SPIT =
      new AlgaePreset("SPIT", Inches.of(8), Degrees.of(205), RPM.of(1000));
  public static final AlgaePreset ALGAE_DISCARD =
      new AlgaePreset("DISCARD", Inches.of(8), Degrees.of(170), RPM.of(1500));
  public static final AlgaePreset ALGAE_STOW =
      new AlgaePreset("STOW", Inches.of(8), Degrees.of(80), RPM.of(0));
  public static final AlgaePreset ALGAE_POPSTICLE =
      new AlgaePreset("POPSTICLE", Inches.of(9), Degrees.of(210), RPM.of(0));

  public static final AlgaePreset ALGAE_NETFROMREEF =
      new AlgaePreset("ALGAE_NETFROMREEF", Inches.of(45), Degrees.of(110), RPM.of(1600));
  public static final AlgaePreset ALGAE_NETFROM6FT =
      new AlgaePreset("ALGAE_NETFROM6FT", Inches.of(8), Degrees.of(180 - 74), RPM.of(1433));
  public static final AlgaePreset ALGAE_NETFROM10FT =
      new AlgaePreset("ALGAE_NETFROM10FT", Inches.of(8), Degrees.of(180 - 66), RPM.of(3000));

  public static final AlgaePreset ALGAE_SHOOTBARGE_OURSIDE =
      new AlgaePreset(
          "ALGAE_SHOOTBARGE_OURSIDE",
          new Pose2d(6.9, 6.0, new Rotation2d(Units.degreesToRadians(0))),
          Inches.of(45),
          Degrees.of(110),
          RPM.of(1600));

  public static final AlgaePreset ALGAE_SHOOTBARGE_THEIRSIDE =
      new AlgaePreset(
          "ALGAE_SHOOTBARGE_THEIRSIDE",
          new Pose2d(11, 7.0, new Rotation2d(Units.degreesToRadians(-180))),
          Inches.of(45),
          Degrees.of(110),
          RPM.of(1600));

  public static final AlgaePreset ALGAE_SHOOTLOW_OURSIDE =
      new AlgaePreset(
          "ALGAE_SHOOTLOW_OURSIDE",
          new Pose2d(7, 7, new Rotation2d(Units.degreesToRadians(-45))),
          Inches.of(12),
          Degrees.of(105),
          RPM.of(2700));

  public static final AlgaePreset ALGAE_SHOOTLOW_THEIRSIDE =
      new AlgaePreset(
          "ALGAE_SHOOTLOW_THEIRSIDE",
          new Pose2d(10.5, 7, new Rotation2d(Units.degreesToRadians(-135))),
          Inches.of(12),
          Degrees.of(105),
          RPM.of(2700));

  public static final AlgaePreset ALGAE_PROCESS_OURSIDE =
      new AlgaePreset(
          "ALGAE_PROCESS_OURSIDE",
          new Pose2d(6.08, 0.65, new Rotation2d(Units.degreesToRadians(-90))),
          Inches.of(8),
          Degrees.of(170),
          RPM.of(700));

  public static final AlgaePreset ALGAE_PROCESS_THEIRSIDE =
      new AlgaePreset(
          "ALGAE_PROCESS_THEIRSIDE",
          new Pose2d(11.48, 7.34, new Rotation2d(Units.degreesToRadians(90))),
          Inches.of(8),
          Degrees.of(170),
          RPM.of(700));

  public static final CoralPreset CORAL_L1 = new CoralPreset("L1", RPM.of(900), RPM.of(1000));
  public static final CoralPreset CORAL_L2 = new CoralPreset("L2", RPM.of(900), RPM.of(1000));
  public static final CoralPreset CORAL_L3 = new CoralPreset("L3", RPM.of(550), RPM.of(1335));
  public static final CoralPreset CORAL_L4 =
      new CoralPreset("L4", RPM.of(1300), RPM.of(1300), Inches.of(7));
}
