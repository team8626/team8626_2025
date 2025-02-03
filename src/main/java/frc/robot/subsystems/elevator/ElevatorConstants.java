package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

  // TODO: Add constants for the elevator
  // Motor Constants
  public static final int CANID = 1;

  // Elevator Constants
  public static final double gearRatio = 15.0 / 1; // TODO: Update to correct value
  public static final double drumRadiusMeters =
      Units.inchesToMeters(1); // TODO: Update to correct value
  public static final double positionConversionFactor = 1.0; // TODO: Update to correct value
  public static final double velocityConversionFactor = 1.0; // TODO: Update to correct value
  public static final double minHeightMeters = Units.inchesToMeters(0);
  public static final double maxHeightMeters = Units.inchesToMeters(60);
  public static final double carriageMassKg = Units.lbsToKilograms(1);
  public static final double velocityMetersPerSecond = Units.inchesToMeters(30);
  public static final double tolerance = Units.inchesToMeters(0.5);

  // PID Constants
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double FF = 0.0;

  // Elevator states
  public class ElevatorStates {
    public static enum ElevatorState {
      STOPPED("idle"),
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
