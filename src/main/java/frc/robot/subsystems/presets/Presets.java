package frc.robot.subsystems.presets;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Presets {

  public static final AlgaePreset ALGAE_LOW =
      new AlgaePreset("LOW", Inches.of(40.5), Degrees.of(200), RPM.of(-1000.0));
  public static final AlgaePreset ALGAE_HIGH =
      new AlgaePreset("HIGH", Inches.of(55), Degrees.of(190), RPM.of(-1000));
  public static final AlgaePreset ALGAE_FLOOR =
      new AlgaePreset("FLOOR", Inches.of(8), Degrees.of(195), RPM.of(-1000));
  public static final AlgaePreset ALGAE_SPIT =
      new AlgaePreset("SPIT", Inches.of(8), Degrees.of(205), RPM.of(1000));
  public static final AlgaePreset ALGAE_PROCESS =
      new AlgaePreset("PROCESS", Inches.of(8), Degrees.of(170), RPM.of(500));
  public static final AlgaePreset ALGAE_STOW =
      new AlgaePreset("STOW", Inches.of(9), Degrees.of(80), RPM.of(0));
  public static final AlgaePreset ALGAE_POPSTICLE =
      new AlgaePreset("POPSTICLE", Inches.of(8), Degrees.of(205), RPM.of(0));

  public static final AlgaePreset ALGAE_NETFROMREEF =
      new AlgaePreset("ALGAE_NETFROMREEF", Inches.of(50), Degrees.of(110), RPM.of(1600));
  public static final AlgaePreset ALGAE_NETFROM6FT =
      new AlgaePreset("ALGAE_NETFROM6FT", Inches.of(8), Degrees.of(180 - 74), RPM.of(1433));
  public static final AlgaePreset ALGAE_NETFROM10FT =
      new AlgaePreset("ALGAE_NETFROM10FT", Inches.of(8), Degrees.of(180 - 66), RPM.of(3000));

  public static final AlgaePreset ALGAE_Test2 =
      new AlgaePreset("ALGAE_Test2", Inches.of(20), Degrees.of(180 - 74), RPM.of(2500));
  public static final AlgaePreset ALGAE_Test3 =
      new AlgaePreset("ALGAE_Test3", Inches.of(20), Degrees.of(180 - 80), RPM.of(2500));
  public static final AlgaePreset ALGAE_Test4 =
      new AlgaePreset("ALGAE_Test4", Inches.of(8), Degrees.of(180 - 80), RPM.of(3000));
  public static final AlgaePreset ALGAE_Test1 =
      new AlgaePreset("ALGAE_Test1", Inches.of(8), Degrees.of(180 - 80), RPM.of(2700));

  public static final AlgaePreset ALGAE_SHOOTBARGE_OURSIDE =
      new AlgaePreset(
          "ALGAE_SHOOTBARGE_OURSIDE",
          new Pose2d(6.9, 6.0, new Rotation2d(Units.degreesToRadians(0))),
          Inches.of(50),
          Degrees.of(110),
          RPM.of(1600));

  public static final AlgaePreset ALGAE_SHOOTBARGE_THEIRSIDE =
      new AlgaePreset(
          "ALGAE_SHOOTBARGE_THEIRSIDE",
          new Pose2d(10.6, 6.0, new Rotation2d(Units.degreesToRadians(-180))),
          Inches.of(50),
          Degrees.of(110),
          RPM.of(1600));

  public static final CoralPreset CORAL_L1 = new CoralPreset("L1", 900, 1000);
  public static final CoralPreset CORAL_L2 = new CoralPreset("L2", 900, 1000);
  public static final CoralPreset CORAL_L3 = new CoralPreset("L3", 550, 1335);
  public static final CoralPreset CORAL_L4 = new CoralPreset("L4", 1300, 1300);

  public static final Pose2d Test3Pose =
      new Pose2d(7, 5, new Rotation2d(Units.degreesToRadians(37)));
}
