package frc.robot.subsystems.climber;

import frc.robot.RobotConstants;

public class ClimberConstants {

  // Climber Constants
  public static final double minAngleDegrees = 14;
  public static final double maxAngleDegrees = 220;
  public static final double restAngleDegrees = 0;
  public static final double positionConversionFactor = 360; // degrees
  public static final double velocityConversionFactor = 360 / 60; // degrees per second

  public static final double toleranceDegrees = 5;

  // Arm Config
  public static final ArmConfig armConfig =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new ArmConfig(13, 12, (125 * 1.5) / 1);
        case SIMBOT -> new ArmConfig(13, 12, (125 * 1.5) / 1);
        default -> new ArmConfig(0, 12, (125 * 1.5) / 1);
      };

  // PID Constants
  public static final Gains gains =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new Gains(0.01, 0.0, 0.0, 0.12, 0.00635, 0);
        case SIMBOT -> new Gains(0.05, 0.0, 0.0, 0.12, 0.00635, 0.0);
        default -> new Gains(0.05, 0.0, 0.0, 0.12, 0.00635, 0.0);
      };

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record ArmConfig(int CANID, double armLengthInches, double reduction) {}
}
