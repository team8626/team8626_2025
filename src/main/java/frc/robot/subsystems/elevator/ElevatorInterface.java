package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.elevator.ElevatorConstants.gains;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates.ElevatorState;

/**
 * The xxxInterface is an interface that defines the methods that must be implemented by any
 * subsystem of type xxx.
 *
 * <p>It contains the default methods that must be implemented by any subsystem of type xxx and the
 * default values that are common to all subsystems of type xxx.
 */
public interface ElevatorInterface {

  public static class ElevatorValues {
    protected ElevatorState state = ElevatorState.IDLE;
    protected boolean isEnabled = false;
    protected boolean isZeroed = false;
    protected Distance currentHeight = Inches.of(0); // Inches
    protected Distance desiredHeight = Inches.of(0); // Inches
    protected Current ampsLeft = Amps.of(0); // Amps
    protected Current ampsRight = Amps.of(0); // Amps
    protected double appliedOutputLeft = 0;
    protected double appliedOutputRight = 0;
    protected Temperature temperatureLeft = Celsius.of(0);
    protected Temperature temperatureRight = Celsius.of(0);

    protected double positionRight = 0;
    protected LinearVelocity velocityRight = MetersPerSecond.of(0);
    protected double voltageRight = 0;

    protected double kP = gains.kP();
    protected double kI = gains.kI();
    protected double kD = gains.kD();
  }

  public default void updateInputs(ElevatorValues values) {}

  public abstract Distance getHeight();

  public abstract void goUp(Distance offset);

  public abstract void goDown(Distance offset);

  public abstract void setHeight(Distance height);

  default void setPID(double kP, double kI, double kD) {}

  public abstract void reset();

  public default void runCharacterization(double input) {}

  // For Characterizarion
  public default void setVoltageMainMotor(double voltage) {}
}
