package frc.robot.subsystems.coralshooter;

import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;

public class CoralShooterConstants {

  // Tuned Values
  public static final double shooterRPM = 2500;
  public static final double intakeRPM = -500;
  public static final double launcherShootRPM = 1000;
  public static final double launcherIntakeRPM = -100;

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

  // Flywheel Config
  public static final FlywheelConfig flywheelConfig =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new FlywheelConfig(4, 0, (3.0 / 1.0), 2 * momentOfInertia, 6000.0);
        case DEVBOT -> new FlywheelConfig(5, 4, (3.0 / 1.0), 2 * momentOfInertia, 6000.0);
        case SIMBOT -> new FlywheelConfig(0, 0, (3.0 / 1.0), 2 * momentOfInertia, 6000.0);
        default -> new FlywheelConfig(5, 4, (1.0 / 2.0), 2 * momentOfInertia, 6000.0);
      };

  // Launcher FLywheel Config
  public static final FlywheelConfig launcherConfig =
      new FlywheelConfig(1, 0, (1.0 / 1.0), 2 * momentOfInertia, 6000.0);

  // InfraRed Port (Sensor to check if the CORAL is loaded
  public static final int infraRedPort = 0; // DIO

  // PID Constants
  public static final Gains gains =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new Gains(0.05, 0.0, 0.0, 0.3, 0.001, 0);
        case DEVBOT -> new Gains(0.05, 0.0, 0.0, 0.3, 0.001, 0.0);
        case SIMBOT -> new Gains(0.05, 0.0, 0.0, 0.01, 0.001, 0.0);
        default -> new Gains(0.05, 0.0, 0.0, 0.3, 0.001, 0.0);
      };

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record FlywheelConfig(
      int leftCANID,
      int rightCANID,
      double reduction,
      double momentOfInertia,
      double maxAcclerationRpmPerSec) {}
}
