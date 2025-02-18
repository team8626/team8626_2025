package frc.robot.subsystems.coralshooter;

public interface CoralShooterInterface {
  void startShooter(double new_RPM, double new_RPMRight);

  void updateRPMShooter(double new_RPM, double new_RPMRight);

  void stopShooter();

  void startLauncher(double new_Setpoint);

  void updateSetpointLauncher(double new_Setpoint);

  void stopLauncher();

  boolean isLoaded();

  double getRPMLeft();

  double getRPMRight();

  double getRPMLauncher();

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

    protected double currentRPMLeft = 0; // RPM
    protected double currentRPMRight = 0; // RPM
    protected double currentRMPLauncher = 0; // RPM
    protected double currentLauncherSetpoint = 0; // [-1;1]

    protected double ampsLeft = 0;
    protected double ampsRight = 0;
    protected double ampsLauncher = 0;

    protected boolean isLoaded = false;
    protected double desiredRPMLeft = CoralShooterConstants.RPMShootLeft;
    protected double desiredRPMRight = CoralShooterConstants.RPMShoolLeft;

    protected double kPLeft = 0.05;
    protected double kILeft = 0.0;
    protected double kDLeft = 0.0;

    protected double kPRight = 0.05;
    protected double kIRight = 0.0;
    protected double kDRight = 0.0;
  }
}
