package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.robot.RobotConstants;

public class WristConstants {

  public static final Angle minAngle = Degrees.of(80);
  public static final Angle maxAngle = Degrees.of(235);
  public static final Angle restAngle = Degrees.of(80);
  public static final Angle positionConversionFactor = Degrees.of(360);
  public static final AngularVelocity velocityConversionFactor =
      DegreesPerSecond.of(360 / 60); // degrees per second

  public static final int maxCurrent = 40; // Amps

  public static final Angle tolerance = Degrees.of(4);

  // Gear box ratio
  public static final double inputGear = 16;
  public static final double stageOneInput = 56;
  public static final double stageOneOutput = 45;
  public static final double stageTwoInput = 40;
  public static final double stageTwoOutput = 16;
  public static final double outputGear = 45;
  public static final double maxPlanetaryRatio = 16 / 1;
  public static final double gearBoxRatio =
      (stageOneInput / inputGear)
          * (stageTwoInput / stageOneOutput)
          * (outputGear / stageTwoOutput)
          * maxPlanetaryRatio;

  public static final Distance armLength = Inches.of(9.5);
  public static final Mass armMass = Pounds.of(13.5);

  // Wrist Config
  public static final WristConfig wristConfig =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new WristConfig(4, gearBoxRatio, Inches.of(9.5));
        case SIMBOT -> new WristConfig(4, gearBoxRatio, Inches.of(9.5));
        default -> new WristConfig(0, gearBoxRatio, Inches.of(9.5));
      };

  // PID Constants
  public static final Gains gains =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new Gains(0.005, 0.0, 0.00006, 0.12, 0.00635, 0);
        case SIMBOT -> new Gains(6, 0.0, 0.00000, 0.12, 0.00635, 0.0);
        default -> new Gains(0.05, 0.0, 0.0, 0.12, 0.00635, 0.0);
      };

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record WristConfig(int CANID, double reduction, Distance armLength) {}
}
