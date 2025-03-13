package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;

public class ElevatorConstants {
  // Elevator Constants
  public static final double gearRatio = 12.0 / 1;
  // public static final double drumRadiusInches = 1.0 / 2;
  public static final double drumRadiusInches = 2.05 / 2;
  public static final double cascadingRatio = 3;

  public static final double positionConversionFactor =
      1 / (Units.inchesToMeters(2 * Math.PI * drumRadiusInches) * cascadingRatio);
  // public static final double positionConversionFactor =
  //     (2 * Math.PI * drumRadiusInches) * cascadingRatio;
  public static final double velocityConversionFactor = positionConversionFactor / 60;
  public static final double minHeightInches = 8;
  public static final double maxHeightInches = 47;
  public static final double initHeightInches = 8;

  public static final int maxCurrent = 40; // Amps

  public static final double carriageMassKg = Units.lbsToKilograms(20);
  public static final double velocityMetersPerSecond = Units.inchesToMeters(30);

  public static final double maxVelocityInchesPerSec = 5;
  public static final double maxAccelerationInchesPerSec2 = 5;
  public static final double toleranceInches = 1;

  public static final MotorConfig motorConfig =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new MotorConfig(13, 1);
        case SIMBOT -> new MotorConfig(13, 1);
        default -> new MotorConfig(13, 1);
      };

  public static final Gains gains =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new Gains(0.5, 0.0, 0.0, 0.43, 3.07, 0.04);
        case SIMBOT -> new Gains(0.5, 0.0, 0.0, 0.43, 3.07, 0.04);
        default -> new Gains(0.05, 0.0, 0.0, 0.43, 3.07, 0.04);
      };

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record MotorConfig(int CANIdLeft, int CANIdRight) {}

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
