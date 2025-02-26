package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.gains;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.subsystems.CS_InterfaceBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates.ElevatorState;

public class Elevator_LinearSparkMax implements ElevatorInterface, CS_InterfaceBase {
  // Declare the subsystem specific hardware here
  // Example:
  private double desiredHeight = ElevatorConstants.minHeightInches;

  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;
  private final SparkClosedLoopController controller;
  private final RelativeEncoder encoder;

  ElevatorFeedforward elevatorFF = new ElevatorFeedforward(gains.kS(), gains.kV(), gains.kA());

  private boolean isEnabled = false;

  public Elevator_LinearSparkMax() {

    // Seup the encoder configuratiion
    // encoderConfig
    //     .setSparkMaxDataPortConfig()
    //     .countsPerRevolution(8192) // RevRobotics Through Bore Encoder
    //     .positionConversionFactor(ElevatorConstants.positionConversionFactor)
    //     .velocityConversionFactor(ElevatorConstants.velocityConversionFactor);

    // Setup configuration for the motor
    motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(ElevatorConstants.maxCurrent);
    // .voltageCompensation(12.0)
    // .softLimit
    // .forwardSoftLimit(Units.inchesToMeters(ElevatorConstants.minHeightInches))
    // .forwardSoftLimitEnabled(true);

    // TODO: add reverse limit too

    motorConfig
        .alternateEncoder
        .positionConversionFactor(ElevatorConstants.positionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.velocityConversionFactor)
        .inverted(true)
        .countsPerRevolution(2048);
    // .countsPerRevolution(8192);
    // .averageDepth(2);

    // apply(encoderConfig).inverted(false).idleMode(IdleMode.kBrake);

    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .positionWrappingEnabled(false)
        .p(gains.kP(), ClosedLoopSlot.kSlot1)
        .i(gains.kI(), ClosedLoopSlot.kSlot1)
        .d(gains.kD(), ClosedLoopSlot.kSlot1)
        .outputRange(-.4, .4, ClosedLoopSlot.kSlot1);

    motorConfig
        .closedLoop
        .maxMotion
        .maxAcceleration(ElevatorConstants.maxAccelerationInchesPerSec2, ClosedLoopSlot.kSlot1)
        .maxVelocity(ElevatorConstants.maxVelocityInchesPerSec, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(ElevatorConstants.toleranceInches, ClosedLoopSlot.kSlot1);

    // motorConfig
    //     .signals
    //     .externalOrAltEncoderPositionAlwaysOn(true)
    //     .externalOrAltEncoderPosition(20)
    //     .externalOrAltEncoderVelocityAlwaysOn(true)
    //     .externalOrAltEncoderVelocity(20)
    //     .appliedOutputPeriodMs(20)
    //     .busVoltagePeriodMs(20)
    //     .outputCurrentPeriodMs(20);

    // Create the motor and assign configuration
    motor = new SparkMax(ElevatorConstants.CANID, MotorType.kBrushless);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create the encoder
    encoder = motor.getAlternateEncoder();
    encoder.setPosition(ElevatorConstants.minHeightInches);

    // Setup the controller
    controller = motor.getClosedLoopController();
    controller.setReference(0, ControlType.kDutyCycle);
    desiredHeight = ElevatorConstants.initHeightInches;
  }

  @Override
  public void updateInputs(ElevatorValues values) {

    values.currentHeight = getElevatorHeight();
    values.desiredHeight = desiredHeight;
    values.amps = motor.getOutputCurrent();
    values.temperature = motor.getMotorTemperature();
    values.isEnabled = isEnabled;

    if (encoder.getVelocity() > 0) {
      values.state = ElevatorState.MOVINGUP;
      values.isEnabled = true;
    } else if (encoder.getVelocity() < 0) {
      values.state = ElevatorState.MOVINGDOWN;
      values.isEnabled = true;
    } else if (motor.getOutputCurrent() > 0) {
      values.state = ElevatorState.HOLDING;
      values.isEnabled = false;
    } else {
      values.state = ElevatorState.IDLE;
      values.isEnabled = false;
    }

    // controller.setReference(desiredHeight, ControlType.kPosition, ClosedLoopSlot.kSlot1);

    controller.setReference(
        desiredHeight,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot1,
        elevatorFF.calculate(desiredHeight),
        ArbFFUnits.kVoltage);
  }

  private double getElevatorHeight() {
    return encoder.getPosition();
  }

  @Override
  public double getHeightInches() {
    double retval = encoder.getPosition();
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
  public void setHeightInches(double new_heightInches) {
    desiredHeight =
        MathUtil.clamp(
            new_heightInches, ElevatorConstants.minHeightInches, ElevatorConstants.maxHeightInches);
  }

  @Override
  public void goUp(double offsetInches) {
    desiredHeight += offsetInches;
  }

  @Override
  public void goDown(double offsetInches) {
    desiredHeight -= offsetInches;
  }
}
