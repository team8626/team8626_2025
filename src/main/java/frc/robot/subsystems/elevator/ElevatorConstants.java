package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {

  // Elevator Constants
  public final static double gearRatio = 15/1; // 15:1
  public final static double minHeightMeters = Units.inchesToMeters(0);
  public final static double maxHeightMeters = Units.inchesToMeters(60);
  public final static double carriageMassKg = Units.lbsToKilograms(50);
  public final static double drumRadiusMeters = Units.inchesToMeters(1);

  // PID Constants
  public final static double kP = 1.0;
  public final static double kI = 0.0;
  public final static double kD = 0.0;
  public final static double FF = 0.0;

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
