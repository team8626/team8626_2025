package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.wrist.WristConstants.gains;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public interface WristInterface {
  void setAngle(Angle newAngle);

  Angle getAngle();

  void goUp(Angle offset);

  void goDown(Angle offset);

  /** Run arm pivot at voltage */
  default void runCharacterization(double input) {}

  default void setPID(double kP, double kI, double kD) {}

  default void updateInputs(WristValues values) {
    // Default implementation
  }

  public class WristValues {
    protected boolean isEnabled = false;
    protected Angle currentAngle = Degrees.of(0); // Degrees
    protected Current amps = Amps.of(0);
    protected Angle desiredAngle = Degrees.of(0); // degrees
    protected double appliedOutput = 0;
    protected Temperature temperature = Celsius.of(0); // Celsius

    // protected double desiredAngleDegrees = 0;

    protected double kP = gains.kP();
    protected double kI = gains.kI();
    protected double kD = gains.kD();
  }
}
