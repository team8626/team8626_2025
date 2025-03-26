package frc.robot.subsystems.coralshooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.coralshooter.CoralShooterConstants.gainsLeft;
import static frc.robot.subsystems.coralshooter.CoralShooterConstants.gainsRight;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public interface CoralShooterInterface {
  void startShooter(AngularVelocity new_RPM, AngularVelocity new_RPMRight);

  void updateRPMShooter(AngularVelocity new_RPM, AngularVelocity new_RPMRight);

  void stopShooter();

  void startLauncher(double new_Setpoint);

  void updateSetpointLauncher(double new_Setpoint);

  void stopLauncher();

  boolean isLoaded();

  AngularVelocity getRPMLeft();

  AngularVelocity getRPMRight();

  AngularVelocity getRPMLauncher();

  double getSetpointLauncher();

  /** Run flywheels at voltage */
  default void runCharacterizationLeft(double input) {}

  default void runCharacterizationRight(double input) {}

  default void setPIDLeft(double kP, double kI, double kD) {}

  default void setPIDRight(double kP, double kI, double kD) {}

  default void updateInputs(CoralShooterValues values) {
    // Default implementation
  }

  public class CoralShooterValues {
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
    protected AngularVelocity desiredRPMLeft;
    protected AngularVelocity desiredRPMRight;

    protected double kPLeft = gainsLeft.kP();
    protected double kILeft = gainsLeft.kI();
    protected double kDLeft = gainsLeft.kD();

    protected double kPRight = gainsRight.kP();
    protected double kIRight = gainsRight.kI();
    protected double kDRight = gainsRight.kD();
  }
}
