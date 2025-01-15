package frc.robot.subsystems.elevator;

import frc.robot.subsystems.CS_InterfaceBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates.ElevatorState;

public class Elevator_LinearSparkMax implements ElevatorInterface, CS_InterfaceBase {
  // Declare the subsystem specific hardware here
  // Example:
  // private final SparkMax motor1;
  // private final SparkMaxConfig config1;

  private boolean is_enabled = false;
  private ElevatorState current_state = ElevatorState.STOPPED;

  public Elevator_LinearSparkMax() {
    
    // Instantiante and initilaze the substem according to Specific1 configuration.
    // This is specific to that robot (Motors, Sensors, controllers...)
    // Example:

    // Setup configuration for the motor
    // config1 = new SparkMaxConfig();
    // config1.inverted(false);

    // Create the motor and assign configuration
    // motor1 = new SparkMax(DummyConstants.kvalue1, MotorType.kBrushless);
    // motor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ElevatorValues values) {
    values.current_height = getElevatorHeight();
    values.state = current_state;
    values.is_enabled = is_enabled;
  }

  @Override
  public void stopElevator() {
    setElevatorSpeed(0);
    current_state = ElevatorState.STOPPED;
  }

  @Override
  public void setElevatorSpeed(double new_speed) {
    // TODO: Set the speed of the elevator motor(s) to new_speed
    if(new_speed > 0) {
      current_state = ElevatorState.MOVINGUP;
    } else if(new_speed < 0) {
      current_state = ElevatorState.MOVINGDOWN;
    } else {
      current_state = ElevatorState.STOPPED;
    }
    is_enabled = (new_speed != 0);
  }

  @Override
  public double getElevatorHeight() {
    double retval = 0.0;

    // TODO: Get the height of the elevator from the sensor
    // retval = ...

    return retval;
  }

  @Override
  public void setElevatorkP(double kP) {
    printf("New kP: %f\n", kP);
  }

  @Override
  public void setElevatorkI(double kI) {
    printf("New kI: %f\n", kI);
  }

  @Override
  public void setElevatorkD(double kD) {
    printf("New kD: %f\n", kD);
  }

  @Override
  public void setElevatorFF(double ff) {
    printf("New FF: %f\n", ff);
  }
}
