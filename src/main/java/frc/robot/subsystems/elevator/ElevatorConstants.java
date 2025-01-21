package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

  // Elevator Constants
  public static final double gearRatio = 15 / 1; // 15:1
  public static final double minHeightMeters = Units.inchesToMeters(0);
  public static final double maxHeightMeters = Units.inchesToMeters(60);
  public static final double carriageMassKg = Units.lbsToKilograms(50);
  public static final double drumRadiusMeters = Units.inchesToMeters(1);

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
