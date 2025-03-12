package frc.robot.subsystems.presets;

public class Presets {

  public static final AlgaePreset ALGAE_LOW = new AlgaePreset("LOW", 36, 190, -600);
  public static final AlgaePreset ALGAE_HIGH = new AlgaePreset("HIGH", 48, 190, -600);
  public static final AlgaePreset ALGAE_FLOOR = new AlgaePreset("FLOOR", 8, 205, -1000);
  public static final AlgaePreset ALGAE_SPIT = new AlgaePreset("SPIT", 8, 205, -1000);
  public static final AlgaePreset ALGAE_PROCESS = new AlgaePreset("PROCESS", 8, 170, 1000);
  public static final AlgaePreset ALGAE_NETFROMREEF =
      new AlgaePreset("ALGAE_NETFROMREEEF", 48, 115, 2200);
  public static final AlgaePreset ALGAE_NETFROM6FT =
      new AlgaePreset("ALGAE_NETFROM6FT", 8, 180 - 74, 1433);
  public static final AlgaePreset ALGAE_NETFROM10FT =
      new AlgaePreset("ALGAE_NETFROM10FT", 8, 180 - 66, 2200);

  public static final AlgaePreset ALGAE_STOW = new AlgaePreset("STOW", 8, 75, 0);

  public static final CoralPreset CORAL_L1 = new CoralPreset("L1", 900, 1000);
  public static final CoralPreset CORAL_L2 = new CoralPreset("L2", 900, 1000);
  public static final CoralPreset CORAL_L3 = new CoralPreset("L3", 550, 1335);
  public static final CoralPreset CORAL_L4 = new CoralPreset("L4", 1300, 1300);
}
