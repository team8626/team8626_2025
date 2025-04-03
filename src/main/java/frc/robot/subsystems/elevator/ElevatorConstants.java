package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import frc.robot.RobotConstants;

public class ElevatorConstants {
  // Elevator Constants
  public static final double gearRatio = 12.0 / 1;
  public static final Distance drumRadius = Inches.of(1.92 / 2);
  public static final double cascadingRatio = 3;
  public static final Distance cascadingOffset = Inches.of(8);

  public static final Distance positionConversionFactor =
      drumRadius.times(Math.PI * 2).times(cascadingRatio);

  public static final LinearVelocity velocityConversionFactor =
      MetersPerSecond.of(positionConversionFactor.in(Meters) / 60);

  public static final Distance minHeight = Inches.of(8);
  public static final Distance maxHeight = Inches.of(58);
  public static final Distance initHeight = Inches.of(8);

  public static final int maxCurrent = 40; // Amps

  public static final Mass carriageMass = Kilogram.of(20);
  public static final LinearVelocity velocity = InchesPerSecond.of(30);

  public static final LinearVelocity maxVelocity = MetersPerSecond.of(1);
  public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(2);
  public static final Distance tolerance = Inches.of(1);

  public static final MotorConfig motorConfig =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new MotorConfig(13, 1);
        case SIMBOT -> new MotorConfig(13, 1);
        default -> new MotorConfig(13, 1);
      };

  public static final Gains gains =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new Gains(0.5, 0.0, 0.0, 0.2261, 5.7, 0.04, -0.85, 0.20);
          // case COMPBOT -> new Gains(0.5, 0.0, 0.0, 0.2261, 3.07, 0.04);
        case SIMBOT -> new Gains(0.5, 0.0, 0.0, 0.2261, 3.07, 0.04, -0.85, 0.20);
        default -> new Gains(0.05, 0.0, 0.0, 0.43, 3.07, 0.04, -0.85, 0.20);
      };

  public record Gains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double minOutput /* Down */,
      double maxOutput /* Up */) {}

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
