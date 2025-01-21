package frc.robot.subsystems.elevator;

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
    protected ElevatorState state = ElevatorState.STOPPED;
    protected boolean is_enabled = false;
    protected double current_height = 0; // Inches

    protected double kP = ElevatorConstants.kP;
    protected double kI = ElevatorConstants.kI;
    protected double kD = ElevatorConstants.kD;
    protected double FF = ElevatorConstants.FF;
  }

  public default void updateInputs(ElevatorValues values) {}

  public abstract void stopElevator();

  public abstract void setElevatorSpeed(double new_speed);

  public abstract double getElevatorHeight();

  public abstract void setElevatorkP(double kP);

  public abstract void setElevatorkI(double kI);

  public abstract void setElevatorkD(double kD);

  public abstract void setElevatorFF(double ff);
}
