package frc.robot.subsystems.wrist;

import static frc.robot.subsystems.wrist.WristConstants.gains;

public interface WristInterface {
  void setAngleDegrees(double new_Angle);

  double getAngleDegrees();

  void goUp(double offsetDegrees);

  void goDown(double offsetDegrees);

  /** Run arm pivot at voltage */
  default void runCharacterization(double input) {}

  default void setPID(double kP, double kI, double kD) {}

  default void updateInputs(WristValues values) {
    // Default implementation
  }

  public class WristValues {
    protected boolean isEnabled = false;
    protected double currentAngleDegrees = 0; // Degrees
    protected double amps = 0;
    protected double desiredAngleDegrees = 0; // degrees
    protected double appliedOutput = 0;
    protected double temperature = 0; // Celsius

    // protected double desiredAngleDegrees = 0;

    protected double kP = gains.kP();
    protected double kI = gains.kI();
    protected double kD = gains.kD();
  }
}
