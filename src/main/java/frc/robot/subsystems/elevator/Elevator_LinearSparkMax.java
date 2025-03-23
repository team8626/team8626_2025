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

  private final SparkMax rightMotor;
  private final SparkMax leftMotor;
  private final SparkMaxConfig rightConfig;
  private final SparkMaxConfig leftConfig;
  private final SparkClosedLoopController rightController;
  private final RelativeEncoder encoder;

  ElevatorFeedforward elevatorFF = new ElevatorFeedforward(gains.kS(), gains.kV(), gains.kA());

  private boolean isEnabled = false;
  private boolean isZeroed = false;
  private boolean isZeroing = false;

  public Elevator_LinearSparkMax() {

    // Setup configuration for the motor
    rightConfig = new SparkMaxConfig();
    rightConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(ElevatorConstants.maxCurrent);

    rightConfig
        .alternateEncoder
        .positionConversionFactor(ElevatorConstants.positionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.velocityConversionFactor)
        .inverted(true)
        .countsPerRevolution(8192);

    rightConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .positionWrappingEnabled(false)
        .p(gains.kP(), ClosedLoopSlot.kSlot1)
        .i(gains.kI(), ClosedLoopSlot.kSlot1)
        .d(gains.kD(), ClosedLoopSlot.kSlot1)
        .outputRange(-0.6, 0.45, ClosedLoopSlot.kSlot1); // Down, Up

    rightConfig
        .closedLoop
        .maxMotion
        .maxAcceleration(ElevatorConstants.maxAccelerationInchesPerSec2, ClosedLoopSlot.kSlot1)
        .maxVelocity(ElevatorConstants.maxVelocityInchesPerSec, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(ElevatorConstants.toleranceInches, ClosedLoopSlot.kSlot1);

    // Create the motor and assign configuration
    rightMotor = new SparkMax(motorConfig.CANIdRight(), MotorType.kBrushless);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Setup configuration for the follower motor
    leftConfig = new SparkMaxConfig();
    leftConfig
        .idleMode(IdleMode.kBrake)
        .follow(rightMotor, false)
        .smartCurrentLimit(ElevatorConstants.maxCurrent);

    leftMotor = new SparkMax(motorConfig.CANIdLeft(), MotorType.kBrushless);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create the encoder
    encoder = rightMotor.getAlternateEncoder();
    encoder.setPosition(ElevatorConstants.minHeightInches);

    // Setup the controller
    rightController = rightMotor.getClosedLoopController();
    rightController.setReference(0, ControlType.kDutyCycle);

    // Zero the elvator
    this.reset();
  }

  @Override
  public void updateInputs(ElevatorValues values) {

    values.currentHeight = this.getElevatorHeight();
    values.desiredHeight = this.desiredHeightInches;
    values.ampsLeft = this.leftMotor.getOutputCurrent();
    values.ampsRight = this.rightMotor.getOutputCurrent();
    values.temperatureLeft = this.leftMotor.getMotorTemperature();
    values.temperatureRight = this.rightMotor.getMotorTemperature();
    values.appliedOutputLeft = this.leftMotor.getAppliedOutput();
    values.appliedOutputRight = this.rightMotor.getAppliedOutput();
    values.isEnabled = this.isEnabled;
    values.isZeroed = this.isZeroed;

    if (encoder.getVelocity() > 0) {
      values.state = ElevatorState.MOVINGUP;
      values.isEnabled = true;
    } else if (encoder.getVelocity() < 0) {
      values.state = ElevatorState.MOVINGDOWN;
      values.isEnabled = true;
    } else if (rightMotor.getOutputCurrent() > 0) {
      values.state = ElevatorState.HOLDING;
      values.isEnabled = false;
    } else {
      values.state = ElevatorState.IDLE;
      values.isEnabled = false;
    }

    // controller.setReference(desiredHeight, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    if (this.isZeroed) {
      // rightController.setReference(
      //   desiredHeightInches,
      //   ControlType.kPosition,
      //   ClosedLoopSlot.kSlot1,
      //   elevatorFF.calculate(desiredHeightInches),
      //   ArbFFUnits.kVoltage);
      rightController.setReference(
          desiredHeightInches,
          ControlType.kPosition,
          ClosedLoopSlot.kSlot1,
          elevatorFF.calculate(desiredHeightInches),
          ArbFFUnits.kVoltage);
    } else {
      if (this.isZeroing) {
        if (this.rightMotor.getOutputCurrent() > 30) {
          this.desiredHeightInches = ElevatorConstants.initHeightInches;
          encoder.setPosition(ElevatorConstants.initHeightInches);
          rightController.setReference(0, ControlType.kDutyCycle);

          this.isZeroed = true;
          this.isZeroing = false;
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
    rightController.setReference(-.1, ControlType.kDutyCycle);
  }

  @Override
  public double getHeightInches() {
    double retval = encoder.getPosition();
    return retval;
  }

  @Override
  public void setPID(double newkP, double newkI, double newkD) {
    printf("New PID: %f, %f, %f", newkP, newkI, newkD);

    leftConfig.closedLoop.pid(newkP, newkI, newkD, ClosedLoopSlot.kSlot1);
    rightConfig.closedLoop.pid(newkP, newkI, newkD, ClosedLoopSlot.kSlot1);
    leftMotor.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
