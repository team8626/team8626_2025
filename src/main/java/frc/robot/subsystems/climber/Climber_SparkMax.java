package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.flywheelConfig;
import static frc.robot.subsystems.climber.ClimberConstants.gains;

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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.CS_InterfaceBase;

public class Climber_SparkMax implements ClimberInterface, CS_InterfaceBase {

  private SparkMax motor;

  private SparkMaxConfig config;

  private final SparkClosedLoopController controller;
  private final RelativeEncoder encoder;

  // done by ai, idk if it works, but it makes public Climber_SparkMax() not throw an error
  SimpleMotorFeedforward FF = new SimpleMotorFeedforward(gains.kS(), gains.kV(), gains.kA());

  private boolean climberIsEnabled = false;

  public Climber_SparkMax() {
    // Setup configuration for the left motor
    config = new SparkMaxConfig();
    config.inverted(false).idleMode(IdleMode.kBrake);

    config
        .encoder
        .positionConversionFactor(1 / flywheelConfig.reduction())
        .velocityConversionFactor(1 / flywheelConfig.reduction());

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control.
        .p(ClimberConstants.gains.kP())
        .i(ClimberConstants.gains.kI())
        .d(ClimberConstants.gains.kD())
        // .velocityFF(0.002)
        .outputRange(-1, 1);

    motor = new SparkMax(flywheelConfig.leftCANID(), MotorType.kBrushless);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller = motor.getClosedLoopController();
    encoder = motor.getEncoder();
    controller.setReference(0, ControlType.kDutyCycle);

    // Setup configuration for the right motor (follower)
    config = new SparkMaxConfig();
    config.follow(motor, true);

    motor = new SparkMax(flywheelConfig.rightCANID(), MotorType.kBrushless);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Launcher Motor
    config = new SparkMaxConfig();
    config.inverted(false);
  }

  @Override
  public void updateInputs(ClimberValues values) {
    values.climberIsEnabled = climberIsEnabled;

    values.currentAngleDegrees = getAngleDegrees();

    values.amps = motor.getOutputCurrent();
  }

  public double getAngleDegrees() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAngleDegrees'");
  }

  // added to fix error at top of class (im hope this doesn't break anything)
  @Override
  public void setAngleDegrees(double new_angle) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setAngleDegrees'");
  }

  @Override
  public void startClimber(double new_RPM) {
    controller.setReference(
        new_RPM,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        FF.calculate(new_RPM),
        ArbFFUnits.kVoltage);

    climberIsEnabled = true;
  }

  @Override
  public void stopClimber() {
    controller.setReference(0, ControlType.kDutyCycle);
    climberIsEnabled = false;
  }

  @Override
  public void updateClimberRPM(double new_RPM) {
    if (climberIsEnabled) {
      controller.setReference(
          new_RPM,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          FF.calculate(new_RPM),
          ArbFFUnits.kVoltage);
    }
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

  @Override
  public double getClimberRPM() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getClimberRPM'");
  }
}
