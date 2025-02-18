package frc.robot.subsystems.wrist;

import static frc.robot.subsystems.wrist.WristConstants.gains;
import static frc.robot.subsystems.wrist.WristConstants.wristConfig;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.CS_InterfaceBase;
import frc.robot.subsystems.climber.ClimberConstants;

public class Wrist_SparkFlex implements WristInterface, CS_InterfaceBase {

  private SparkFlex motor;
  private SparkFlexConfig config;

  private final SparkClosedLoopController controller;
  private AbsoluteEncoder encoder;
  private final AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
  private double setPointDegrees;

  SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(gains.kS(), gains.kV(), gains.kA());

  private boolean isEnabled = false;

  public Wrist_SparkFlex() {
    // Setup configuration for the encoder
    encoderConfig
        .positionConversionFactor(ClimberConstants.positionConversionFactor)
        .velocityConversionFactor(ClimberConstants.velocityConversionFactor);

    // Setup configuration for the motor
    config = new SparkFlexConfig();

    config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // Set PID values for position control.
        .p(ClimberConstants.gains.kP())
        .i(ClimberConstants.gains.kI())
        .d(ClimberConstants.gains.kD())
        .outputRange(-1, 1);

    config.apply(encoderConfig);

    motor = new SparkFlex(wristConfig.CANID(), MotorType.kBrushless);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getAbsoluteEncoder();

    controller = motor.getClosedLoopController();
    controller.setReference(0, ControlType.kDutyCycle);
  }

  @Override
  public void updateInputs(WristValues values) {
    values.isEnabled = isEnabled;
    values.currentAngleDegrees = getAngleDegrees();
    values.amps = motor.getOutputCurrent();

    controller.setReference(setPointDegrees, ControlType.kPosition);
  }

  public double getAngleDegrees() {
    return encoder.getPosition();
  }

  // added to fix error at top of class (im hope this doesn't break anything)
  @Override
  public void setAngleDegrees(double new_angle) {
    setPointDegrees = new_angle;
  }

  // @Override
  // public void start(double new_setpoint) {
  //   controller.setReference(new_setpoint, ControlType.kDutyCycle);
  //   isEnabled = true;
  // }

  // @Override
  // public void stop() {
  //   controller.setReference(0, ControlType.kDutyCycle);
  //   isEnabled = false;
  // }

  @Override
  public void setPID(double newkP, double newkI, double newkD) {
    config.closedLoop.p(newkP).i(newkI).d(newkD);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    printf("New PID: %f, %f, %f", newkP, newkI, newkD);
  }

  @Override
  public void runCharacterization(double input) {
    motor.setVoltage(input);
  }
}
