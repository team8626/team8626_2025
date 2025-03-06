package frc.robot.subsystems.algaeshooter;

import static frc.robot.subsystems.algaeshooter.AlgaeShooterConstants.gains;

public interface AlgaeShooterInterface {
  void startShooter(double new_RPM);

  void updateShooterRPM(double new_RPM);

  void stopShooter();

  void startLauncher(double new_Setpoint);

  void updateLauncherSetpoint(double new_Setpoint);

  void stopLauncher();

  boolean shooterIsLoaded();

  double getShooterRPMLeft();

  double getShooterRPMRight();

  double getLauncherRPM();

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

    protected double currentRPMLeft = 0; // RPM
    protected double currentRPMRight = 0; // RPM
    protected double currentRMPLauncher = 0; // RPM
    protected double currentLauncherSetpoint = 0; // [-1;1]

    protected double ampsLeft = 0;
    protected double ampsRight = 0;
    protected double ampsLauncher = 0;

    protected double tempLeft = 0;
    protected double tempRight = 0;
    protected double tempLauncher = 0;

    protected double appliedOutputLeft = 0;
    protected double appliedOutputRight = 0;
    protected double appliedOutputLauncher = 0;

    protected boolean isLoaded = false;
    protected double desiredRPM = AlgaeShooterConstants.shootRPM;

    protected double kP = gains.kP();
    protected double kI = gains.kI();
    protected double kD = gains.kD();
  }
}
