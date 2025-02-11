package frc.robot.subsystems.climber;

public interface ClimberInterface {
  double getAngleDegrees();

  void setAngleDegrees(double new_angle);

  void startClimber(double new_RPM);

  void stopClimber();

  void updateClimberRPM(double new_RPM);

  double getClimberRPM();

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

    //  protected double desiredAngle = ClimberConstants.shootRPM;

    protected double kP = 0.05;
    protected double kI = 0.0;
    protected double kD = 0.0;
    protected double FF = 0.0;

    public double currentRPM;
  }
}
