package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.CS_InterfaceBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates.ElevatorState;

public class Elevator_LinearSparkMax implements ElevatorInterface, CS_InterfaceBase {
  // Declare the subsystem specific hardware here
  // Example:

  private final SparkMax motor1;
  private final SparkMaxConfig config1;
  private final SparkClosedLoopController controller1;
  private final RelativeEncoder encoder1;

  private boolean is_enabled = false;
  private ElevatorState current_state = ElevatorState.STOPPED;

  public Elevator_LinearSparkMax() {

    // Setup configuration for the motor
    config1 = new SparkMaxConfig();
    config1.inverted(false).idleMode(IdleMode.kBrake);

    // Create the motor and assign configuration
    motor1 = new SparkMax(ElevatorConstants.CANID, MotorType.kBrushless);
    motor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create the encoder
    encoder1 = motor1.getAlternateEncoder();
    // encoder1.setPositionConversionFactor(ElevatorConstants.positionConversionFactor)
    //  .setVelocityConversionFactor(ElevatorConstants.velocityConversionFactor);
    // TODO: Set the position and velocity conversion factors for the encoder

    // Setup the controller
    controller1 = motor1.getClosedLoopController();
    controller1.setReference(0, ControlType.kDutyCycle);
  }

  @Override
  public void updateInputs(ElevatorValues values) {
    values.current_height = getElevatorHeight();
    values.state = current_state;
    values.is_enabled = is_enabled;
  }

  private double getElevatorHeight() {
    return encoder1.getPosition();
  }

  @Override
  public void stopElevator() {
    setElevatorSpeed(0);
    current_state = ElevatorState.STOPPED;
  }

  @Override
  public void setElevatorSpeed(double new_speed) {
    // TODO: Set the speed of the elevator motor(s) to new_speed
    if (new_speed > 0) {
      current_state = ElevatorState.MOVINGUP;
    } else if (new_speed < 0) {
      current_state = ElevatorState.MOVINGDOWN;
    } else {
      current_state = ElevatorState.STOPPED;
    }
    is_enabled = (new_speed != 0);
  }

  @Override
  public double getHeightInches() {
    double retval = encoder1.getPosition();

    // TODO: Get the height of the elevator from the sensor
    // retval = ...

    return retval;
  }

  @Override
  public void setElevatorkP(double new_value) {
    printf("New kP: %f\n", new_value);
  }

  @Override
  public void setElevatorkI(double new_value) {
    printf("New kI: %f\n", new_value);
  }

  @Override
  public void setElevatorkD(double new_value) {
    printf("New kD: %f\n", new_value);
  }

  @Override
  public void setElevatorFF(double new_value) {
    printf("New FF: %f\n", new_value);
  }

  @Override
  public void moveInches(double offsetInches) {
    controller1.setReference(offsetInches + this.getElevatorHeight(), ControlType.kPosition);
  }

  @Override
  public void setHeightInches(double heightInches) {
    encoder1.setPosition((heightInches));
  }
}
