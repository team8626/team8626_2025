package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.armConfig;
import static frc.robot.subsystems.climber.ClimberConstants.gains;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.CS_InterfaceBase;

public class Climber_SparkMax implements ClimberInterface, CS_InterfaceBase {

  private SparkMax motor;

  private SparkMaxConfig config;

  private final SparkClosedLoopController controller;
  private AbsoluteEncoder encoder;
  private final AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
  private double setPointDegrees = ClimberConstants.minAngleDegrees;

  // done by ai, idk if it works, but it makes public Climber_SparkMax() not throw an error
  SimpleMotorFeedforward FF = new SimpleMotorFeedforward(gains.kS(), gains.kV(), gains.kA());

  private boolean climberIsEnabled = false;

  public Climber_SparkMax() {
    // Setup configuration for the encoder
    encoderConfig
        .inverted(false)
        .positionConversionFactor(ClimberConstants.positionConversionFactor)
        .velocityConversionFactor(ClimberConstants.velocityConversionFactor);

    // Setup configuration for the motor
    config = new SparkMaxConfig();

    config.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(false)
        // .positionWrappingInputRange(0, 360)
        // Set PID values for position control.
        .p(ClimberConstants.gains.kP())
        .i(ClimberConstants.gains.kI())
        .d(ClimberConstants.gains.kD())
        .outputRange(-1, 1);

    config
        .softLimit
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(ClimberConstants.minAngleDegrees)
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(ClimberConstants.maxAngleDegrees);

    config.apply(encoderConfig);

    motor = new SparkMax(armConfig.CANID(), MotorType.kBrushless);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getAbsoluteEncoder();

    controller = motor.getClosedLoopController();
    controller.setReference(0, ControlType.kDutyCycle);
  }

  @Override
  public void updateInputs(ClimberValues values) {
    values.climberIsEnabled = climberIsEnabled;
    values.currentAngleDegrees = getAngleDegrees();
    values.amps = motor.getOutputCurrent();
    values.desiredAngleDegrees = setPointDegrees;

    controller.setReference(setPointDegrees, ControlType.kPosition);
  }

  public double getAngleDegrees() {
    return encoder.getPosition();
  }

  // added to fix error at top of class (im hope this doesn't break anything)
  @Override
  public void setAngleDegrees(double new_angle) {
    setPointDegrees = new_angle;
    setPointDegrees =
        MathUtil.clamp(
            new_angle, ClimberConstants.minAngleDegrees, ClimberConstants.maxAngleDegrees);
  }

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
