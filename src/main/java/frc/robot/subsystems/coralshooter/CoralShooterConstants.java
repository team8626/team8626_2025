package frc.robot.subsystems.coralshooter;

import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;

public class CoralShooterConstants {

  // Tuned Values
  // public static final double RPMShootLeft = 1275;
  // public static final double RPMShootRight = 1275;
  public static final double RPMIntake = -300;
  public static final double launcherShootSetpoint = -1.0;
  public static final double launcherIntakeSetpoint = 0.3;

  public static final double RPMTolerance = 20;
  public static final double RPMDifferentialTolerance = 10;
  public static final double launchTimerSeconds = 0.2;
  public static final double shooterTimerSeconds = 0.2;

  public static final int maxCurrent = 60; // Amps

  // CoralShooter Constants
  public static final double wheelRadiusMeters = Units.inchesToMeters(2);
  public static final double wheelMassKg = Units.lbsToKilograms(0.03); // am-2647
  // private static final double stealthWheelMomentOfInertia = 0.5 * wheelMassKg * wheelRadiusMeters
  // * wheelRadiusMeters;
  private static final double momentOfInertia = 1;

  // Flywheel Config
  public static final FlywheelConfig flywheelConfig =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new FlywheelConfig(12, 2, (3.0 / 1.0), 2 * momentOfInertia, 6000.0);
        case SIMBOT -> new FlywheelConfig(12, 2, (3.0 / 1.0), 2 * momentOfInertia, 6000.0);
        default -> new FlywheelConfig(0, 0, (3.0 / 1.0), 2 * momentOfInertia, 6000.0);
      };

  // Launcher FLywheel Config
  public static final FlywheelConfig launcherConfig =
      new FlywheelConfig(3, 0, (1.0 / 1.0), 2 * momentOfInertia, 6000.0);

  // Lidar Port (Sensor to check if the CORAL is loaded)
  public static final int lidarPort = 0; // DIO
  public static final int leftUSPort = 1; // Analog
  public static final int rightUSPort = 2; // Analog

  public static double usScaleFactor = 5 * 1 / (5. / 1024.); // scale converting voltage to distance

  // PID Constants
  public static final Gains gainsLeft =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new Gains(0.0001, 0.0, 0.0, 0.10133, 0.00612, 0);
        case SIMBOT -> new Gains(0.05, 0.0, 0.0, 0.12, 0.00635, 0.0);
        default -> new Gains(0.05, 0.0, 0.0, 0.12, 0.00635, 0.0);
      };
  public static final Gains gainsRight =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new Gains(0.0001, 0.0, 0.0, 0.11083, 0.00608, 0);
        case SIMBOT -> new Gains(0.05, 0.0, 0.0, 0.12, 0.00635, 0.0);
        default -> new Gains(0.05, 0.0, 0.0, 0.12, 0.00635, 0.0);
      };

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record FlywheelConfig(
      int CANIdLeft,
      int CANIdRight,
      double reduction,
      double momentOfInertia,
      double maxAcclerationRpmPerSec) {}
}
