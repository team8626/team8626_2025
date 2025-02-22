package frc.robot.subsystems.coralshooter;

import static frc.robot.subsystems.coralshooter.CoralShooterConstants.gainsLeft;
import static frc.robot.subsystems.coralshooter.CoralShooterConstants.gainsRight;

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

    protected double kPLeft = gainsLeft.kP();
    protected double kILeft = gainsLeft.kI();
    protected double kDLeft = gainsLeft.kD();

    protected double kPRight = gainsRight.kP();
    protected double kIRight = gainsRight.kI();
    protected double kDRight = gainsRight.kD();
  }
}
