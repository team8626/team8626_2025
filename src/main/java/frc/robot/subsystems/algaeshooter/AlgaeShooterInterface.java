package frc.robot.subsystems.algaeshooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.algaeshooter.AlgaeShooterConstants.gains;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public interface AlgaeShooterInterface {
  void startShooter(AngularVelocity new_RPM);
  void startShooter(AngularVelocity new_RPM);

  void updateShooterRPM(AngularVelocity new_RPM);

  void stopShooter();

  void startLauncher(double new_Setpoint);

  void startShooterBySetpoint(double new_Setpoint);

  void updateLauncherSetpoint(double new_Setpoint);

  void stopLauncher();

  boolean shooterIsLoaded();

  AngularVelocity getShooterRPMLeft();

  AngularVelocity getShooterRPMRight();

  AngularVelocity getLauncherRPM();

  double getLauncherSetpoint();

  /** Run flywheels at voltage */
  default void runCharacterizationLeft(double input) {}

  default void runCharacterizationRight(double input) {}

  default void setPID(double kP, double kI, double kD) {}

  default void updateInputs(AlgaeShooterValues values) {
    // Default implementation
  }

  public class AlgaeShooterValues {
    protected boolean launchIsEnabled = false;
    protected boolean shooterIsEnabled = false;

    protected AngularVelocity currentRPMLeft = RPM.of(0);
    protected AngularVelocity currentRPMRight = RPM.of(0);
    protected AngularVelocity currentRMPLauncher = RPM.of(0);
    protected double currentLauncherSetpoint = 0; // [-1;1]

    protected Current ampsLeft = Amps.of(0);
    protected Current ampsRight = Amps.of(0);
    protected Current ampsLauncher = Amps.of(0);

    protected Temperature tempLeft = Celsius.of(0);
    protected Temperature tempRight = Celsius.of(0);
    protected Temperature tempLauncher = Celsius.of(0);

    protected double appliedOutputLeft = 0;
    protected double appliedOutputRight = 0;
    protected double appliedOutputLauncher = 0;

    protected boolean isLoaded = false;
    protected AngularVelocity desiredRPM = RPM.of(0);

    protected double kP = gains.kP();
    protected double kI = gains.kI();
    protected double kD = gains.kD();
  }
}
