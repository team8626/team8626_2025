package frc.robot.subsystems.presets;

public class Presets {

  public static final AlgaePreset ALGAE_LOW = new AlgaePreset("LOW", 36, 190, -600);
  public static final AlgaePreset ALGAE_HIGH = new AlgaePreset("HIGH", 48, 190, -600);
  public static final AlgaePreset ALGAE_FLOOR = new AlgaePreset("FLOOR", 8, 205, -1000);
  public static final AlgaePreset ALGAE_PROCESS = new AlgaePreset("PROCESS", 8, 170, 1000);
  public static final AlgaePreset ALGAE_NETFROMREEF =
      new AlgaePreset("SHOOT_FROM_REEF", 10, 115, 2200);
  public static final AlgaePreset ALGAE_STOW = new AlgaePreset("STOW", 8, 75, 0);

  public static final CoralPreset CORAL_L1 = new CoralPreset("L1", 900, 1000);
  public static final CoralPreset CORAL_L2 = new CoralPreset("L2", 900, 1000);
  public static final CoralPreset CORAL_L3 = new CoralPreset("L3", 550, 1335);
  public static final CoralPreset CORAL_L4 = new CoralPreset("L4", 1275, 1275);
}
