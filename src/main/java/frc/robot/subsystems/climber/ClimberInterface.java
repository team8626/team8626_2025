package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.gains;

public interface ClimberInterface {
  double getAngleDegrees();

  void setAngleDegrees(double new_angle);

  // void start(double new_setpoint);

  // void stop();

  /** Run flywheels at voltage */
  default void runCharacterization(double input) {}

  default void setPID(double kP, double kI, double kD) {}

  default void updateInputs(ClimberValues values) {
    // Default implementation
  }

  public class ClimberValues {
    protected boolean climberIsEnabled = false;
    protected double currentAngleDegrees = 0; // degrees
    protected double amps = 0;
    protected double desiredAngleDegrees = 0; // degrees

    //  protected double desiredAngle = ClimberConstants.shootRPM;

    protected double kP = gains.kP();
    protected double kI = gains.kI();
    protected double kD = gains.kD();

    public double currentRPM;
  }
}
