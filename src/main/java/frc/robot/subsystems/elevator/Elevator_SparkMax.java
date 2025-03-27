package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.gains0;
import static frc.robot.subsystems.elevator.ElevatorConstants.gains1;
import static frc.robot.subsystems.elevator.ElevatorConstants.motorConfig;
import static frc.utils.CS_Utils.clamp;

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
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.CS_InterfaceBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates.ElevatorState;

public class Elevator_SparkMax implements ElevatorInterface, CS_InterfaceBase {

  private Distance desiredHeight = ElevatorConstants.minHeight;

  private final SparkMax rightMotor;
  private final SparkMax leftMotor;
  private final SparkMaxConfig rightConfig;
  private final SparkMaxConfig leftConfig;
  private final SparkClosedLoopController rightController;
  private final RelativeEncoder encoder;

  ElevatorFeedforward elevatorFF0 = new ElevatorFeedforward(gains0.kS(), gains0.kG(), gains0.kV());
  ElevatorFeedforward elevatorFF1 = new ElevatorFeedforward(gains1.kS(), gains1.kG(), gains1.kV());

  private boolean isEnabled = false;
  private boolean isZeroed = false;
  private boolean isZeroing = false;
  private Timer zeroingTimer = new Timer();

  public Elevator_SparkMax() {

    // Setup configuration for the motor
    rightConfig = new SparkMaxConfig();
    rightConfig
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .smartCurrentLimit(ElevatorConstants.maxCurrent);

    rightConfig
        .alternateEncoder
        .positionConversionFactor(ElevatorConstants.positionConversionFactor.in(Meters))
        .velocityConversionFactor(ElevatorConstants.velocityConversionFactor.in(MetersPerSecond))
        .inverted(true)
        .countsPerRevolution(8192);

    // rightConfig
    //     .closedLoop
    //     .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
    //     .positionWrappingEnabled(false);
    // .p(gains0.kP(), ClosedLoopSlot.kSlot0)
    // .i(gains0.kI(), ClosedLoopSlot.kSlot0)
    // .d(gains0.kD(), ClosedLoopSlot.kSlot0)
    // .outputRange(-0.85, 0.20, ClosedLoopSlot.kSlot0) // Down, Up
    // .p(gains1.kP(), ClosedLoopSlot.kSlot1)
    // .i(gains1.kI(), ClosedLoopSlot.kSlot1)
    // .d(gains1.kD(), ClosedLoopSlot.kSlot1)
    // .outputRange(-0.85, 0.20, ClosedLoopSlot.kSlot1); // Down, Up

    // maxMotionConfig
    //     .maxAcceleration(
    //         ElevatorConstants.maxAcceleration.in(InchesPerSecondPerSecond),
    // ClosedLoopSlot.kSlot0)
    //     .maxVelocity(ElevatorConstants.maxVelocity.in(InchesPerSecond), ClosedLoopSlot.kSlot0)
    //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0)
    //     .allowedClosedLoopError(ElevatorConstants.tolerance.in(Inches), ClosedLoopSlot.kSlot0)
    //     .maxAcceleration(
    //         ElevatorConstants.maxAcceleration.in(InchesPerSecondPerSecond),
    // ClosedLoopSlot.kSlot1)
    //     .maxVelocity(ElevatorConstants.maxVelocity.in(InchesPerSecond), ClosedLoopSlot.kSlot1)
    //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot1)
    //     .allowedClosedLoopError(ElevatorConstants.tolerance.in(Inches), ClosedLoopSlot.kSlot1);

    // Setup the MAXMotion configuration
    rightConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .positionWrappingEnabled(false)
        .outputRange(
            gains0.minOutput(),
            gains0.maxOutput(),
            ClosedLoopSlot.kSlot0) // Down, Up  --- Slot0, Going Down
        .outputRange(
            gains1.minOutput(),
            gains1.maxOutput(),
            ClosedLoopSlot.kSlot1) // Down, Up  --- Slot1, Going Up
        .maxMotion
        .maxAcceleration(
            ElevatorConstants.maxAcceleration.in(MetersPerSecondPerSecond), ClosedLoopSlot.kSlot0)
        .maxVelocity(ElevatorConstants.maxVelocity.in(MetersPerSecond), ClosedLoopSlot.kSlot0)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0)
        .allowedClosedLoopError(ElevatorConstants.tolerance.in(Meter), ClosedLoopSlot.kSlot0)
        .maxAcceleration(
            ElevatorConstants.maxAcceleration.in(MetersPerSecondPerSecond), ClosedLoopSlot.kSlot1)
        .maxVelocity(ElevatorConstants.maxVelocity.in(MetersPerSecond), ClosedLoopSlot.kSlot1)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(ElevatorConstants.tolerance.in(Meter), ClosedLoopSlot.kSlot1);

    // rightConfig
    //   .softLimit
    //   .forwardSoftLimitEnabled(true)
    //
    // .forwardSoftLimit(ElevatorConstants.maxHeight.minus(ElevatorConstants.cascadingOffset).in(Meters))
    //   .reverseSoftLimitEnabled(true)
    //
    // .reverseSoftLimit(ElevatorConstants.minHeight.minus(ElevatorConstants.cascadingOffset).in(Meters));

    // Create the motor and assign configuration
    rightMotor = new SparkMax(motorConfig.CANIdRight(), MotorType.kBrushless);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Setup configuration for the follower motor
    leftConfig = new SparkMaxConfig();
    leftConfig
        .idleMode(IdleMode.kCoast)
        .follow(rightMotor, false)
        .smartCurrentLimit(ElevatorConstants.maxCurrent);

    leftMotor = new SparkMax(motorConfig.CANIdLeft(), MotorType.kBrushless);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create the encoder
    encoder = rightMotor.getAlternateEncoder();
    encoder.setPosition(ElevatorConstants.minHeight.in(Meters));

    // Setup the controller
    rightController = rightMotor.getClosedLoopController();
    rightController.setReference(0, ControlType.kDutyCycle);

    // Zero the elevator
    this.reset();
  }

  @Override
  public void updateInputs(ElevatorValues values) {
    values.currentHeight = getHeight();
    values.currentVelocity = MetersPerSecond.of(encoder.getVelocity());
    values.desiredHeight = this.desiredHeight;
    values.ampsLeft = Amps.of(this.leftMotor.getOutputCurrent());
    values.ampsRight = Amps.of(this.rightMotor.getOutputCurrent());
    values.voltsLeft = Volts.of(this.leftMotor.getAppliedOutput() * 12);
    values.voltsRight = Volts.of(this.rightMotor.getAppliedOutput() * 12);
    values.temperatureLeft = Celsius.of(this.leftMotor.getMotorTemperature());
    values.temperatureRight = Celsius.of(this.rightMotor.getMotorTemperature());
    values.appliedOutputLeft = this.leftMotor.getAppliedOutput();
    values.appliedOutputRight = this.rightMotor.getAppliedOutput();
    values.absolutePositionRight = Meters.of(encoder.getPosition());
    values.velocityRight = MetersPerSecond.of(encoder.getVelocity());

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

    if (this.isZeroed) {
      boolean goingUp = desiredHeight.gt(this.getHeight());
      ClosedLoopSlot slot = goingUp ? ClosedLoopSlot.kSlot0 : ClosedLoopSlot.kSlot1;
      ElevatorFeedforward ff = goingUp ? elevatorFF0 : elevatorFF1;

      rightController.setReference(
          desiredHeight.minus(ElevatorConstants.cascadingOffset).in(Meters),
          ControlType.kMAXMotionPositionControl,
          slot,
          ff.calculate(desiredHeight.in(Meters)),
          ArbFFUnits.kVoltage);
    } else {
      if (this.isZeroing) {
        if (this.rightMotor.getOutputCurrent() > ElevatorConstants.stallCurrent.in(Amps)) {

          this.zeroingTimer.start();
          rightController.setReference(0, ControlType.kDutyCycle);

          if (this.zeroingTimer.hasElapsed(ElevatorConstants.zeroingTime.in(Seconds))) {
            this.zeroingTimer.stop();
            this.zeroingTimer.reset();
            this.isZeroed = true;
            this.isZeroing = false;

            this.desiredHeight = ElevatorConstants.initHeight;
            encoder.setPosition(
                (ElevatorConstants.initHeight.minus(ElevatorConstants.cascadingOffset)).in(Meters));
          }
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
    this.zeroingTimer.stop();
    this.zeroingTimer.reset();
    this.isZeroed = false;
    this.isZeroing = true;
    rightController.setReference(-.1, ControlType.kDutyCycle);
  }

  @Override
  public Distance getHeight() {
    Distance retval = (Meters.of(encoder.getPosition())).plus(ElevatorConstants.cascadingOffset);
    return retval;
  }

  // @Override
  // public void setPID0(double newkP, double newkI, double newkD) {
  //   printf("New PID Slot0: %f, %f, %f", newkP, newkI, newkD);

  //   rightConfig.closedLoop.pid(newkP, newkI, newkD, ClosedLoopSlot.kSlot0);
  //   rightMotor.configure(
  //       rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  // }

  // @Override
  // public void setPID1(double newkP, double newkI, double newkD) {
  //   printf("New PID Slot1: %f, %f, %f", newkP, newkI, newkD);

  //   rightConfig.closedLoop.pid(newkP, newkI, newkD, ClosedLoopSlot.kSlot1);
  //   rightMotor.configure(
  //       rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  // }

  // @Override
  // public void setOutputRange0(double min, double max) {
  //   printf("New Range Slot0: %f, %f, %f", min, max);

  //   rightConfig.closedLoop.outputRange(min, max, ClosedLoopSlot.kSlot0);
  //   rightMotor.configure(
  //       rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  // }

  @Override
  public void setOutputRange(double min, double max, ClosedLoopSlot slot) {
    printf("New Range %s: %f, %f, %f", slot.toString(), min, max);

    rightConfig.closedLoop.outputRange(min, max, slot);
    rightMotor.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setPID(double newkP, double newkI, double newkD, ClosedLoopSlot slot) {
    printf("New PID %s: %f, %f, %f", slot.toString(), newkP, newkI, newkD);

    rightConfig.closedLoop.pid(newkP, newkI, newkD, slot);
    rightMotor.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  // @Override
  // public void setOutputRange1(double min, double max) {
  //   printf("New Range Slot1: %f, %f, %f", min, max);

  //   rightConfig.closedLoop.outputRange(min, max, ClosedLoopSlot.kSlot1);
  //   rightMotor.configure(
  //       rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  // }

  @Override
  public void setHeight(Distance new_height) {
    // desiredHeight = clamp(new_height, ElevatorConstants.minHeight, ElevatorConstants.maxHeight);
    // printf(
    //     "Setting Height: %f [%f - %f]\n",
    //     desiredHeight.in(Inches),
    //     ElevatorConstants.minHeight.in(Inches),
    //     ElevatorConstants.maxHeight.in(Inches));
  }

  @Override
  public void goUp(Distance offset) {
    desiredHeight =
        clamp(desiredHeight.plus(offset), ElevatorConstants.minHeight, ElevatorConstants.maxHeight);
  }

  @Override
  public void goDown(Distance offset) {
    desiredHeight =
        clamp(
            desiredHeight.minus(offset), ElevatorConstants.minHeight, ElevatorConstants.maxHeight);
  }

  @Override
  public void setVoltageMainMotor(Voltage voltage) {
    this.rightMotor.set(voltage.in(Volts) / this.rightMotor.getBusVoltage());
  }
}
