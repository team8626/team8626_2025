package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.gains;

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
    protected double currentHeight = 0; // Inches
    protected double desiredHeight = 0; // Inches
    protected double ampsLeft = 0; // Amps
    protected double ampsRight = 0; // Amps
    protected double appliedOutputLeft = 0;
    protected double appliedOutputRight = 0;
    protected double temperatureLeft = 0; // Celsius
    protected double temperatureRight = 0; // Celsius

    protected double kP = gains.kP();
    protected double kI = gains.kI();
    protected double kD = gains.kD();
  }

  public default void updateInputs(ElevatorValues values) {}

  public abstract double getHeightInches();

  public abstract void goUp(double offsetInches);

  public abstract void goDown(double offsetInches);

  public abstract void setHeightInches(double heightInches);

  public abstract void setElevatorkP(double kP);

  public abstract void setElevatorkI(double kI);

  public abstract void setElevatorkD(double kD);

  public abstract void reset();

  public default void runCharacterization(double input) {}
}
