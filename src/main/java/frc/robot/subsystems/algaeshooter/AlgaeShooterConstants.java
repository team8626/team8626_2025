package frc.robot.subsystems.algaeshooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotConstants;

public class AlgaeShooterConstants {

  // Tuned Values
  public static final AngularVelocity shootRPM = RPM.of(2200);
  public static final AngularVelocity intakeRPM = RPM.of(-600);
  public static final AngularVelocity discardRPM = RPM.of(600);
  public static final double launcherShootSetpoint = 1.0;
  public static final double launcherIntakeSetpoint = -0.5;
  public static final double launcherDiscardSetpoint = 0.5;
  public static final double discardShootSetpoint = 0.5;

  public static final int maxCurrent = 50; // Amps

  public static final AngularVelocity shooterRPMTolerance = RPM.of(100);
  public static final double launchTimerSeconds = 0.5;
  public static final double intakeTimerSeconds = 0.5;

  // AlgaeShooter Constants
  public static final Distance wheelRadius = Inches.of(2);
  public static final double wheelMassKg = Units.lbsToKilograms(0.03); // am-2647
  // private static final double stealthWheelMomentOfInertia = 0.5 * wheelMassKg * wheelRadiusMeters
  // * wheelRadiusMeters;
  private static final double momentOfInertia = 1;

  // Flywheel Config
  public static final FlywheelConfig flywheelConfig =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new FlywheelConfig(11, 10, 33.0 / 20.0, 2 * momentOfInertia, 6000.0);
        case SIMBOT -> new FlywheelConfig(11, 10, 33 / 20, 2 * momentOfInertia, 6000.0);
          // case COMPBOT -> new FlywheelConfig(11, 10, (30 / 24), 2 * momentOfInertia, 6000.0);
          // case SIMBOT -> new FlywheelConfig(11, 10, (30 / 24), 2 * momentOfInertia, 6000.0);
        default -> new FlywheelConfig(0, 0, (33 / 20), 2 * momentOfInertia, 6000.0);
      };

  // Launcher FLywheel Config
  public static final FlywheelConfig launcherConfig =
      new FlywheelConfig(9, 0, (36 / 24), 2 * momentOfInertia, 6000.0);

  // InfraRed Port (Sensor to check if the ALGAE is loaded
  public static final int infraRedPort = 1; // DIO

  // PID Constants
  public static final Gains gains =
      switch (RobotConstants.robotType) {
        case COMPBOT -> new Gains(0.0004, 0.0, 0.0, 0.10395, 0.00296, 0);
        case SIMBOT -> new Gains(0.05, 0.0, 0.0, 0.10395, 0.00296, 0.0);
        default -> new Gains(0.05, 0.0, 0.0, 0.10395, 0.00296, 0.0);
      };

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record FlywheelConfig(
      int CANIdLeft,
      int CANIdRight,
      double reduction,
      double momentOfInertia,
      double maxAcclerationRpmPerSec) {}
}
