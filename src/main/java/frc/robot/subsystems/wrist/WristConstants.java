package frc.robot.subsystems.wrist;

import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;

public class WristConstants {
  public static final double defaultAngleDegrees = 0; //TODO get a good value

  public static final double inputGear = 16;
  public static final double stageOneInput = 56;
  public static final double stageOneOutput = 45;
  public static final double stageTwoInput = 40;
  public static final double stageTwoOutput = 16;
  public static final double outputGear = 45;
  public static final double maxPlanetaryRatio = 16 / 1;
  public static final double gearBoxRatio = (stageOneInput / inputGear) * (stageTwoInput / stageOneOutput) * (outputGear / stageTwoOutput) * maxPlanetaryRatio;
  
  public static final double armLength = Units.inchesToMeters(9.5);
  public static final double armMass = Units.lbsToKilograms(13.5);
  public static final double minAngleDegrees = 0;
  public static final double maxAngleDegrees = 180;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  public static final double wristEncoderDistPerPulse = 2.0 * Math.PI / 4096;//TODO is that used irl???
  
  // Tuned Values
  public static final double shootRPM = 1275;
  public static final double intakeRPM = -300;
  public static final double launcherShootSetpoint = -1.0;
  public static final double launcherIntakeSetpoint = 0.3;

  public static final double shooterRPMTolerance = 50;
  public static final double launchTimerSeconds = 0.2;

  // CoralShooter Constants
  public static final double wheelRadiusMeters = Units.inchesToMeters(2);
  public static final double wheelMassKg = Units.lbsToKilograms(0.03); // am-2647
  // private static final double stealthWheelMomentOfInertia = 0.5 * wheelMassKg * wheelRadiusMeters
  // * wheelRadiusMeters;
  private static final double momentOfInertia = 1;

  // PID Constants
  // public final static double kP = 1.0;
  // public final static double kI = 0.0;
  // public final static double kD = 0.0;
  // public final static double FF = 0.0;

  // Wrist Config
  public static final WristConfig wristConfig =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new WristConfig(20, gearBoxRatio, 2 * momentOfInertia, 6000.0);
        case DEVBOT -> new WristConfig(20, gearBoxRatio, 2 * momentOfInertia, 6000.0);
        case SIMBOT -> new WristConfig(0, gearBoxRatio, 2 * momentOfInertia, 6000.0);
        default -> new WristConfig(0, gearBoxRatio, 2 * momentOfInertia, 6000.0);
      };

  // PID Constants
  public static final Gains gains =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new Gains(0.0001, 0.0, 0.0, 0.12, 0.00635, 0);
        case DEVBOT -> new Gains(0.0001, 0.0, 0.0, 0.12, 0.00635, 0.0);
        case SIMBOT -> new Gains(0.05, 0.0, 0.0, 0.12, 0.00635, 0.0);
        default -> new Gains(0.05, 0.0, 0.0, 0.12, 0.00635, 0.0);
      };

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record WristConfig(
      int CANID,
      double reduction,
      double momentOfInertia,
      double maxAcclerationDegreesPerSec) {}
}
