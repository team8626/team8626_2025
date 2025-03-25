package frc.robot.subsystems.presets;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Presets {

  public static final AlgaePreset ALGAE_LOW = new AlgaePreset("LOW", 40.5, 200, -1000);
  public static final AlgaePreset ALGAE_HIGH = new AlgaePreset("HIGH", 55, 190, -1000);
  public static final AlgaePreset ALGAE_FLOOR = new AlgaePreset("FLOOR", 8, 195, -1000);
  public static final AlgaePreset ALGAE_SPIT = new AlgaePreset("SPIT", 8, 205, -1000);
  public static final AlgaePreset ALGAE_PROCESS = new AlgaePreset("PROCESS", 8, 170, 500);
  public static final AlgaePreset ALGAE_STOW = new AlgaePreset("STOW", 9, 80, 0);

  public static final AlgaePreset ALGAE_NETFROMREEF =
      new AlgaePreset("ALGAE_NETFROMREEF", 50, 110, 1600);
  public static final AlgaePreset ALGAE_NETFROM6FT =
      new AlgaePreset("ALGAE_NETFROM6FT", 8, 180 - 74, 1433);
  public static final AlgaePreset ALGAE_NETFROM10FT =
      new AlgaePreset("ALGAE_NETFROM10FT", 8, 180 - 66, 3000);

  public static final AlgaePreset ALGAE_Test2 = new AlgaePreset("ALGAE_Test2", 20, 180 - 74, 2500);
  public static final AlgaePreset ALGAE_Test3 = new AlgaePreset("ALGAE_Test3", 20, 180 - 80, 2500);
  public static final AlgaePreset ALGAE_Test4 = new AlgaePreset("ALGAE_Test4", 8, 180 - 80, 3000);
  public static final AlgaePreset ALGAE_Test1 = new AlgaePreset("ALGAE_Test1", 8, 180 - 80, 2700);

  public static final AlgaePreset ALGAE_SHOOTBARGE_OURSIDE =
      new AlgaePreset(
          "ALGAE_SHOOTBARGE_OURSIDE",
          new Pose2d(6.9, 6.0, new Rotation2d(Units.degreesToRadians(0))),
          50,
          110,
          1600);

  public static final AlgaePreset ALGAE_SHOOTBARGE_THEIRSIDE =
      new AlgaePreset(
          "ALGAE_SHOOTBARGE_THEIRSIDE",
          new Pose2d(10.6, 6.0, new Rotation2d(Units.degreesToRadians(-180))),
          50,
          110,
          1600);

  public static final CoralPreset CORAL_L1 = new CoralPreset("L1", 900, 1000);
  public static final CoralPreset CORAL_L2 = new CoralPreset("L2", 900, 1000);
  public static final CoralPreset CORAL_L3 = new CoralPreset("L3", 550, 1335);
  public static final CoralPreset CORAL_L4 = new CoralPreset("L4", 1300, 1300);

  public static final Pose2d Test3Pose =
      new Pose2d(7, 5, new Rotation2d(Units.degreesToRadians(37)));
}
