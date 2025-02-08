package frc.robot.subsystems.wrist;

public interface WristInterface {
  void setAngleDegrees(double new_Angle);

  double getAngleDegrees();

  /** Run arm pivot at voltage */
  default void runCharacterization(double input) {}

  default void setPID(double kP, double kI, double kD) {}

  default void updateInputs(WristValues values) {
    // Default implementation
  }

  public class WristValues {
    protected boolean wristIsEnabled = false;

    protected double currentAngleDegrees = 0; // Degrees

    protected double amps = 0;

    protected double desiredAngleDegrees = 0;

    protected double kP = 0.05;
    protected double kI = 0.0;
    protected double kD = 0.0;
    protected double FF = 0.0;
  }
}
