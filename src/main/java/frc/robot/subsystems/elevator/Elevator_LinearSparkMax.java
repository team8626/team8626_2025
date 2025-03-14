package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.gains;
import static frc.robot.subsystems.elevator.ElevatorConstants.motorConfig;

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
  private double desiredHeightInches = ElevatorConstants.minHeightInches;

  private final SparkMax motorRight;
  private final SparkMax motorLeft;
  private final SparkMaxConfig motorRightConfig;
  private final SparkMaxConfig motorLeftConfig;
  private final SparkClosedLoopController controllerRight;
  private final RelativeEncoder encoder;

  ElevatorFeedforward elevatorFF = new ElevatorFeedforward(gains.kS(), gains.kV(), gains.kA());

  private boolean isEnabled = false;
  private boolean isZeroed = false;
  private boolean isZeroing = false;

  public Elevator_LinearSparkMax() {

    // Setup configuration for the motor
    motorRightConfig = new SparkMaxConfig();
    motorRightConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(ElevatorConstants.maxCurrent);

    motorRightConfig
        .alternateEncoder
        .positionConversionFactor(ElevatorConstants.positionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.velocityConversionFactor)
        .inverted(true)
        .countsPerRevolution(2048);

    motorRightConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .positionWrappingEnabled(false)
        .p(gains.kP(), ClosedLoopSlot.kSlot1)
        .i(gains.kI(), ClosedLoopSlot.kSlot1)
        .d(gains.kD(), ClosedLoopSlot.kSlot1)
        .outputRange(-.6, .3, ClosedLoopSlot.kSlot1);

    motorRightConfig
        .closedLoop
        .maxMotion
        .maxAcceleration(ElevatorConstants.maxAccelerationInchesPerSec2, ClosedLoopSlot.kSlot1)
        .maxVelocity(ElevatorConstants.maxVelocityInchesPerSec, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(ElevatorConstants.toleranceInches, ClosedLoopSlot.kSlot1);

    // Create the motor and assign configuration
    motorRight = new SparkMax(motorConfig.CANIdRight(), MotorType.kBrushless);
    motorRight.configure(
        motorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Setup configuration for the follower motor
    motorLeftConfig = new SparkMaxConfig();
    motorLeftConfig
        .idleMode(IdleMode.kBrake)
        .follow(motorRight, false)
        .smartCurrentLimit(ElevatorConstants.maxCurrent);

    motorLeft = new SparkMax(motorConfig.CANIdLeft(), MotorType.kBrushless);
    motorLeft.configure(
        motorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create the encoder
    encoder = motorRight.getAlternateEncoder();
    encoder.setPosition(ElevatorConstants.minHeightInches);

    // Setup the controller
    controllerRight = motorRight.getClosedLoopController();
    controllerRight.setReference(0, ControlType.kDutyCycle);

    // Zero the elvator
    this.reset();
  }

  @Override
  public void updateInputs(ElevatorValues values) {

    values.currentHeight = this.getElevatorHeight();
    values.desiredHeight = this.desiredHeightInches;
    values.ampsLeft = this.motorLeft.getOutputCurrent();
    values.ampsRight = this.motorRight.getOutputCurrent();
    values.temperatureLeft = this.motorLeft.getMotorTemperature();
    values.temperatureRight = this.motorRight.getMotorTemperature();
    values.appliedOutputLeft = this.motorLeft.getAppliedOutput();
    values.appliedOutputRight = this.motorRight.getAppliedOutput();
    values.isEnabled = this.isEnabled;
    values.isZeroed = this.isZeroed;

    if (encoder.getVelocity() > 0) {
      values.state = ElevatorState.MOVINGUP;
      values.isEnabled = true;
    } else if (encoder.getVelocity() < 0) {
      values.state = ElevatorState.MOVINGDOWN;
      values.isEnabled = true;
    } else if (motorRight.getOutputCurrent() > 0) {
      values.state = ElevatorState.HOLDING;
      values.isEnabled = false;
    } else {
      values.state = ElevatorState.IDLE;
      values.isEnabled = false;
    }

    // controller.setReference(desiredHeight, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    if (this.isZeroed) {
      controllerRight.setReference(
          desiredHeightInches,
          ControlType.kPosition,
          ClosedLoopSlot.kSlot1,
          elevatorFF.calculate(desiredHeightInches),
          ArbFFUnits.kVoltage);
    } else {
      if (this.isZeroing) {
        if (this.motorRight.getOutputCurrent() > 30) {
          controllerRight.setReference(0, ControlType.kDutyCycle);

          this.isZeroed = true;
          this.isZeroing = false;
          this.desiredHeightInches = ElevatorConstants.initHeightInches;
        }
      } else {
        // Not Zeroed and not Zeroing yet...
        this.reset();
      }
    }
  }

  private double getElevatorHeight() {
    return encoder.getPosition();
  }

  // Zero the elevator by srunning it reverse until it hits the bottom ---Gently---
  public void reset() {
    this.isZeroed = false;
    this.isZeroing = true;
    controllerRight.setReference(-.25, ControlType.kDutyCycle);
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
    desiredHeightInches =
        MathUtil.clamp(
            new_heightInches, ElevatorConstants.minHeightInches, ElevatorConstants.maxHeightInches);
  }

  @Override
  public void goUp(double offsetInches) {
    desiredHeightInches += offsetInches;
  }

  @Override
  public void goDown(double offsetInches) {
    desiredHeightInches -= offsetInches;
  }
}
