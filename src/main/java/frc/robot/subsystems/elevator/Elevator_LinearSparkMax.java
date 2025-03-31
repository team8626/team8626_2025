package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Volts;
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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.CS_InterfaceBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates.ElevatorState;

public class Elevator_LinearSparkMax implements ElevatorInterface, CS_InterfaceBase {

  private Distance desiredHeight = ElevatorConstants.minHeight;

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
        .positionConversionFactor(ElevatorConstants.positionConversionFactor.in(Inches))
        .velocityConversionFactor(ElevatorConstants.velocityConversionFactor.in(InchesPerSecond))
        .inverted(true)
        .countsPerRevolution(8192);

    rightConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .positionWrappingEnabled(false)
        .p(gains.kP(), ClosedLoopSlot.kSlot1)
        .i(gains.kI(), ClosedLoopSlot.kSlot1)
        .d(gains.kD(), ClosedLoopSlot.kSlot1)
        .outputRange(gains.minOutput(), gains.maxOutput(), ClosedLoopSlot.kSlot1); // Down, Up

    // rightConfig
    //     .closedLoop
    //     .maxMotion
    //     .maxAcceleration(
    //         ElevatorConstants.maxAcceleration.in(InchesPerSecondPerSecond),
    // ClosedLoopSlot.kSlot1)
    //     .maxVelocity(ElevatorConstants.maxVelocity.in(InchesPerSecond), ClosedLoopSlot.kSlot1)
    //     .allowedClosedLoopError(ElevatorConstants.tolerance.in(Inches), ClosedLoopSlot.kSlot1);

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
    encoder.setPosition(ElevatorConstants.minHeight.in(Inches));

    // Setup the controller
    rightController = rightMotor.getClosedLoopController();
    rightController.setReference(0, ControlType.kDutyCycle);

    // Zero the elvator
    this.reset();
  }

  @Override
  public void updateInputs(ElevatorValues values) {
    desiredHeight =
        Inches.of(
            MathUtil.clamp(
                desiredHeight.in(Inches),
                ElevatorConstants.minHeight.in(Inches),
                ElevatorConstants.maxHeight.in(Inches)));

    values.currentHeight = getHeight();
    values.currentVelocity = InchesPerSecond.of(encoder.getVelocity());
    values.desiredHeight = this.desiredHeight;
    values.ampsLeft = Amps.of(this.leftMotor.getOutputCurrent());
    values.ampsRight = Amps.of(this.rightMotor.getOutputCurrent());
    values.voltsLeft = Volts.of(this.leftMotor.getAppliedOutput() * this.leftMotor.getBusVoltage());
    values.voltsRight =
        Volts.of(this.rightMotor.getAppliedOutput() * this.rightMotor.getBusVoltage());
    values.temperatureLeft = Celsius.of(this.leftMotor.getMotorTemperature());
    values.temperatureRight = Celsius.of(this.rightMotor.getMotorTemperature());
    values.appliedOutputLeft = this.leftMotor.getAppliedOutput();
    values.appliedOutputRight = this.rightMotor.getAppliedOutput();
    values.absolutePositionRight = Inches.of(encoder.getPosition());
    values.velocityRight = InchesPerSecond.of(encoder.getVelocity());

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
      Distance deltaElevation = desiredHeight.minus(ElevatorConstants.cascadingOffset);
      rightController.setReference(
          deltaElevation.in(Inches),
          // ControlType.kMAXMotionPositionControl,
          ControlType.kPosition,
          ClosedLoopSlot.kSlot1,
          elevatorFF.calculate(deltaElevation.in(Inches)),
          ArbFFUnits.kVoltage);
    } else {
      if (this.isZeroing) {
        if (this.rightMotor.getOutputCurrent() > 30) {
          this.desiredHeight = ElevatorConstants.initHeight;
          encoder.setPosition(
              (ElevatorConstants.initHeight.minus(ElevatorConstants.cascadingOffset)).in(Inches));
          rightController.setReference(0, ControlType.kDutyCycle);

          this.isZeroed = true;
          this.isZeroing = false;
        }
      } else {
        // Not Zeroed and not Zeroing yet...
        this.reset();
      }

      // desired height is low and amps are high, chain skipped,
      // that requires to reset...
      // if ((desiredHeight.in(Inches) < 9) && (this.rightMotor.getOutputCurrent() > 30)) {
      //   // this.desiredHeightInches = ElevatorConstants.initHeightInches;
      //   encoder.setPosition(ElevatorConstants.initHeight.in(Meters));
      //   rightController.setReference(0, ControlType.kDutyCycle);

      //   this.isZeroed = true;
      //   this.isZeroing = false;
      // }
    }
  }

  // Zero the elevator by srunning it reverse until it hits the bottom ---Gently---
  public void reset() {
    this.isZeroed = false;
    this.isZeroing = true;
    rightController.setReference(-.1, ControlType.kDutyCycle);
  }

  @Override
  public Distance getHeight() {
    Distance retval = (Inches.of(encoder.getPosition())).plus(ElevatorConstants.cascadingOffset);
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
  public void outputRange(double newMin, double newMax) {
    printf("New Output Range: %f, %f", newMin, newMax);

    leftConfig.closedLoop.outputRange(newMin, newMax, ClosedLoopSlot.kSlot1);
    leftMotor.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setHeight(Distance new_height) {
    desiredHeight = new_height;

    // printf(
    //     "Setting Height: %f [%f - %f]\n",
    //     desiredHeight.in(Inches),
    //     ElevatorConstants.minHeight.in(Inches),
    //     ElevatorConstants.maxHeight.in(Inches));
  }

  @Override
  public void goUp(Distance offset) {
    desiredHeight = desiredHeight.plus(offset);
  }

  @Override
  public void goDown(Distance offset) {
    desiredHeight = desiredHeight.minus(offset);
  }

  @Override
  public void setVoltageMainMotor(Voltage voltage) {
    this.rightMotor.set(voltage.in(Volts) / this.rightMotor.getBusVoltage());
  }
}
