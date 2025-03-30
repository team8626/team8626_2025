package frc.robot.subsystems.algaeshooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.algaeshooter.AlgaeShooterConstants.flywheelConfig;
import static frc.robot.subsystems.algaeshooter.AlgaeShooterConstants.gains;
import static frc.robot.subsystems.algaeshooter.AlgaeShooterConstants.launcherConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.CS_InterfaceBase;

public class AlgaeShooter_SparkFlex implements AlgaeShooterInterface, CS_InterfaceBase {

  private final SparkFlex leftMotor;
  private final SparkFlexConfig leftConfig;
  private final SparkClosedLoopController leftController;
  private final RelativeEncoder leftEncoder;

  private final SparkFlex rightMotor;
  private final SparkFlexConfig rightConfig;
  private final SparkClosedLoopController rightController;
  private final RelativeEncoder rightEncoder;

  private final SparkFlex launchMotor;
  private final SparkFlexConfig launchConfig;
  private final SparkClosedLoopController launchController;
  private final RelativeEncoder launchEncoder;

  SimpleMotorFeedforward shooterFFLeft =
      new SimpleMotorFeedforward(gains.kS(), gains.kV(), gains.kA());
  SimpleMotorFeedforward shooterFFRight =
      new SimpleMotorFeedforward(gains.kS(), gains.kV(), gains.kA());

  private SparkLimitSwitch loadedSensor;

  private boolean shooterIsEnabled = false;
  private boolean launcherIsEnabled = false;
  private double currentLauncherSetpoint = 0;

  public AlgaeShooter_SparkFlex() {
    // Setup configuration for the left motor
    leftConfig = new SparkFlexConfig();
    leftConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(AlgaeShooterConstants.maxCurrent);

    leftConfig
        .encoder
        .positionConversionFactor(1 / flywheelConfig.reduction())
        .velocityConversionFactor(1 / flywheelConfig.reduction());

    leftConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control.
        .pid(
            AlgaeShooterConstants.gains.kP(),
            AlgaeShooterConstants.gains.kI(),
            AlgaeShooterConstants.gains.kD(),
            ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1);

    // Disable limit switch. otherwise the loaded sensor causes the motor to stop!
    leftConfig.limitSwitch.reverseLimitSwitchEnabled(false).forwardLimitSwitchEnabled(false);

    leftMotor = new SparkFlex(flywheelConfig.CANIdLeft(), MotorType.kBrushless);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftController = leftMotor.getClosedLoopController();
    leftEncoder = leftMotor.getEncoder();
    leftController.setReference(0, ControlType.kDutyCycle);

    // Setup configuration for the right motor
    rightConfig = new SparkFlexConfig();
    rightConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(AlgaeShooterConstants.maxCurrent);

    rightConfig
        .encoder
        .positionConversionFactor(1 / flywheelConfig.reduction())
        .velocityConversionFactor(1 / flywheelConfig.reduction());

    rightConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            AlgaeShooterConstants.gains.kP(),
            AlgaeShooterConstants.gains.kI(),
            AlgaeShooterConstants.gains.kD(),
            ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1);

    rightMotor = new SparkFlex(flywheelConfig.CANIdRight(), MotorType.kBrushless);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightController = rightMotor.getClosedLoopController();
    rightEncoder = rightMotor.getEncoder();
    rightController.setReference(0, ControlType.kDutyCycle);

    // Launcher Motor
    launchConfig = new SparkFlexConfig();
    launchConfig.inverted(false).smartCurrentLimit(AlgaeShooterConstants.maxCurrent);

    launchConfig
        .encoder
        .positionConversionFactor(1 / launcherConfig.reduction())
        .velocityConversionFactor(1 / launcherConfig.reduction());

    launchMotor = new SparkFlex(launcherConfig.CANIdLeft(), MotorType.kBrushless);
    launchMotor.configure(
        launchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    launchController = launchMotor.getClosedLoopController();
    launchEncoder = launchMotor.getEncoder();
    launchController.setReference(0, ControlType.kDutyCycle);

    loadedSensor = leftMotor.getForwardLimitSwitch();
  }

  @Override
  public void updateInputs(AlgaeShooterValues values) {
    values.launchIsEnabled = launcherIsEnabled;
    values.shooterIsEnabled = shooterIsEnabled;

    values.currentRPMLeft = getShooterRPMLeft();
    values.currentRPMRight = getShooterRPMRight();

    values.currentRMPLauncher = getLauncherRPM();
    values.currentLauncherSetpoint = getLauncherSetpoint();

    values.ampsLeft = Amps.of(leftMotor.getOutputCurrent());
    values.ampsRight = Amps.of(rightMotor.getOutputCurrent());
    values.ampsLauncher = Amps.of(launchMotor.getOutputCurrent());

    values.tempLeft = Celsius.of(leftMotor.getMotorTemperature());
    values.tempRight = Celsius.of(rightMotor.getMotorTemperature());
    values.tempLauncher = Celsius.of(launchMotor.getMotorTemperature());

    values.appliedOutputLeft = leftMotor.getAppliedOutput();
    values.appliedOutputRight = rightMotor.getAppliedOutput();
    values.appliedOutputLauncher = launchMotor.getAppliedOutput();

    values.isLoaded = shooterIsLoaded();
  }

  @Override
  public void startShooter(AngularVelocity new_RPM) {
    leftController.setReference(
        new_RPM.in(RPM),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        shooterFFLeft.calculate(new_RPM.in(RPM)),
        ArbFFUnits.kVoltage);

    rightController.setReference(
        new_RPM.in(RPM),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        shooterFFRight.calculate(new_RPM.in(RPM)),
        ArbFFUnits.kVoltage);

    shooterIsEnabled = true;
  }

  @Override
  public void stopShooter() {
    leftController.setReference(0, ControlType.kDutyCycle);
    rightController.setReference(0, ControlType.kDutyCycle);
    shooterIsEnabled = false;
  }

  @Override
  public void updateShooterRPM(AngularVelocity new_RPM) {
    if (shooterIsEnabled) {
      leftController.setReference(
          new_RPM.in(RPM),
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          shooterFFLeft.calculate(new_RPM.in(RPM)),
          ArbFFUnits.kVoltage);

      rightController.setReference(
          new_RPM.in(RPM),
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          shooterFFRight.calculate(new_RPM.in(RPM)),
          ArbFFUnits.kVoltage);
    }
  }

  @Override
  public void stopLauncher() {
    updateLauncherSetpoint(0);
    launcherIsEnabled = false;
  }

  @Override
  public void updateLauncherSetpoint(double new_Setpoint) {
    currentLauncherSetpoint = new_Setpoint;
    if (launcherIsEnabled) {
      launchController.setReference(new_Setpoint, ControlType.kDutyCycle);
    }
  }

  @Override
  public void startLauncher(double new_Setpoint) {
    currentLauncherSetpoint = new_Setpoint;
    launchController.setReference(currentLauncherSetpoint, ControlType.kDutyCycle);
    launcherIsEnabled = true;
  }

  @Override
  public void startShooterBySetpoint(double new_Setpoint) {
    rightController.setReference(new_Setpoint, ControlType.kDutyCycle);
    leftController.setReference(new_Setpoint, ControlType.kDutyCycle);
    shooterIsEnabled = true;
  }

  @Override
  public AngularVelocity getShooterRPMLeft() {
    return RPM.of(leftEncoder.getVelocity());
  }

  @Override
  public AngularVelocity getShooterRPMRight() {
    return RPM.of(rightEncoder.getVelocity());
  }

  @Override
  public AngularVelocity getLauncherRPM() {
    return RPM.of(launchEncoder.getVelocity() / (36 / 24));
  }

  public double getLauncherSetpoint() {
    return currentLauncherSetpoint;
  }

  @Override
  public boolean shooterIsLoaded() {
    return loadedSensor.isPressed();
  }

  @Override
  public void setPID(double newkP, double newkI, double newkD) {
    leftConfig.closedLoop.pid(newkP, newkI, newkD, ClosedLoopSlot.kSlot0);
    rightConfig.closedLoop.pid(newkP, newkI, newkD, ClosedLoopSlot.kSlot0);
    printf("New PID: %f, %f, %f", newkP, newkI, newkD);
    leftMotor.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void runCharacterizationLeft(double input) {
    leftMotor.setVoltage(input);
  }

  @Override
  public void runCharacterizationRight(double input) {
    rightMotor.setVoltage(input);
  }
}
