package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;

public class ElevatorConstants {

  // Motor Constants
  public static final int CANID = 1;

  // Elevator Constants
  public static final double gearRatio = 12.0 / 1;
  public static final double drumRadiusInches = 1.41 / 2;
  public static final double cascadingRatio = 3;

  public static final double positionConversionFactor =
      1 / (Units.inchesToMeters(2 * Math.PI * drumRadiusInches) * cascadingRatio);
  public static final double velocityConversionFactor = positionConversionFactor / 60;
  public static final double minHeightInches = 7;
  public static final double maxHeightInches = 50;

  public static final int maxCurrent = 30; // Amps

  public static final double carriageMassKg = Units.lbsToKilograms(1);
  public static final double velocityMetersPerSecond = Units.inchesToMeters(30);

  public static final double maxVelocityInchesPerSec = 40;
  public static final double maxAccelerationInchesPerSec2 = 20;
  public static final double toleranceInches = 0.25;

  public static final Gains gains =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new Gains(0.5, 0.0, 0.0, 0.12, 0.00635, 0);
        case SIMBOT -> new Gains(0.5, 0.0, 0.0, 0.12, 0.00635, 0.0);
        default -> new Gains(0.05, 0.0, 0.0, 0.12, 0.00635, 0.0);
      };

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  // Elevator states
  public class ElevatorStates {
    public static enum ElevatorState {
      IDLE("idle"),
      HOLDING("holding"),
      MOVINGUP("moving up"),
      MOVINGDOWN("moving up");

      private final String string;

      ElevatorState(String newString) {
        this.string = newString.toUpperCase();
      }

      public String getString() {
        return string.toUpperCase();
      }
    }
  }
}
