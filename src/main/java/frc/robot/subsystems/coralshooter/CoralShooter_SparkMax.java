package frc.robot.subsystems.coralshooter;

import static frc.robot.subsystems.coralshooter.CoralShooterConstants.flywheelConfig;
import static frc.robot.subsystems.coralshooter.CoralShooterConstants.gainsLeft;
import static frc.robot.subsystems.coralshooter.CoralShooterConstants.gainsRight;
import static frc.robot.subsystems.coralshooter.CoralShooterConstants.launcherConfig;

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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.CS_InterfaceBase;

public class CoralShooter_SparkMax implements CoralShooterInterface, CS_InterfaceBase {

  private final SparkMax leftMotor;
  private final SparkMaxConfig leftConfig;
  private final SparkClosedLoopController leftController;
  private final RelativeEncoder leftEncoder;

  private final SparkMax rightMotor;
  private final SparkMaxConfig rightConfig;
  private final SparkClosedLoopController rightController;
  private final RelativeEncoder rightEncoder;

  private final SparkMax launchMotor;
  private final SparkMaxConfig launchConfig;
  private final SparkClosedLoopController launchController;
  private final RelativeEncoder launchEncoder;

  SimpleMotorFeedforward shooterFFLeft =
      new SimpleMotorFeedforward(gainsLeft.kS(), gainsLeft.kV(), gainsLeft.kA());
  SimpleMotorFeedforward shooterFFRight =
      new SimpleMotorFeedforward(gainsRight.kS(), gainsRight.kV(), gainsRight.kA());

  private DigitalInput loadedSensor = new DigitalInput(CoralShooterConstants.lidarPort);

  private boolean shooterIsEnabled = false;
  private boolean launcherIsEnabled = false;
  private double currentLauncherSetpoint = 0;

  public CoralShooter_SparkMax() {
    // Setup configuration for the left motor
    leftConfig = new SparkMaxConfig();
    leftConfig.inverted(false).idleMode(IdleMode.kCoast);

    leftConfig
        .encoder
        .positionConversionFactor(1 / flywheelConfig.reduction())
        .velocityConversionFactor(1 / flywheelConfig.reduction());

    leftConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control.
        .p(CoralShooterConstants.gainsLeft.kP())
        .i(CoralShooterConstants.gainsLeft.kI())
        .d(CoralShooterConstants.gainsLeft.kD())
        // .velocityFF(0.002)
        .outputRange(-1, 1);

    leftMotor = new SparkMax(flywheelConfig.CANIdLeft(), MotorType.kBrushless);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftController = leftMotor.getClosedLoopController();
    leftEncoder = leftMotor.getEncoder();
    leftController.setReference(0, ControlType.kDutyCycle);

    // Setup configuration for the right motor
    rightConfig = new SparkMaxConfig();
    rightConfig.inverted(false).idleMode(IdleMode.kCoast);

    rightConfig
        .encoder
        .positionConversionFactor(1 / flywheelConfig.reduction())
        .velocityConversionFactor(1 / flywheelConfig.reduction());

    rightConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control.
        .p(CoralShooterConstants.gainsLeft.kP())
        .i(CoralShooterConstants.gainsLeft.kI())
        .d(CoralShooterConstants.gainsLeft.kD())
        // .velocityFF(0.002)
        .outputRange(-1, 1);

    rightMotor = new SparkMax(flywheelConfig.CANIdRight(), MotorType.kBrushless);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightController = rightMotor.getClosedLoopController();
    rightEncoder = rightMotor.getEncoder();
    rightController.setReference(0, ControlType.kDutyCycle);

    // Launcher Motor
    launchConfig = new SparkMaxConfig();
    launchConfig.inverted(true);

    launchMotor = new SparkMax(launcherConfig.CANIdLeft(), MotorType.kBrushless);
    launchMotor.configure(
        launchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    launchController = launchMotor.getClosedLoopController();
    launchEncoder = launchMotor.getEncoder();
    launchController.setReference(0, ControlType.kDutyCycle);
  }

  @Override
  public void updateInputs(CoralShooterValues values) {
    values.launchIsEnabled = launcherIsEnabled;
    values.shooterIsEnabled = shooterIsEnabled;

    values.currentRPMLeft = getRPMLeft();
    values.currentRPMRight = getRPMRight();

    values.currentRMPLauncher = getRPMLauncher();
    values.currentLauncherSetpoint = getSetpointLauncher();

    values.ampsLeft = leftMotor.getOutputCurrent();
    values.ampsRight = rightMotor.getOutputCurrent();
    values.ampsLauncher = launchMotor.getOutputCurrent();

    values.isLoaded = isLoaded();
  }

  @Override
  public void startShooter(double new_RPMLeft, double new_RPMRight) {
    leftController.setReference(
        -new_RPMLeft,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        shooterFFLeft.calculate(-new_RPMLeft),
        ArbFFUnits.kVoltage);

    rightController.setReference(
        new_RPMRight,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        shooterFFRight.calculate(new_RPMRight),
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
  public void updateRPMShooter(double new_RPMLeft, double new_RPMRight) {
    if (shooterIsEnabled) {
      leftController.setReference(
          -new_RPMLeft,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          shooterFFLeft.calculate(-new_RPMLeft),
          ArbFFUnits.kVoltage);

      rightController.setReference(
          new_RPMRight,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          shooterFFRight.calculate(new_RPMRight),
          ArbFFUnits.kVoltage);
    }
  }

  @Override
  public void stopLauncher() {
    updateSetpointLauncher(0);
    launcherIsEnabled = false;
  }

  @Override
  public void updateSetpointLauncher(double new_Setpoint) {
    currentLauncherSetpoint = new_Setpoint;
    if (launcherIsEnabled) {
      launchController.setReference(new_Setpoint, ControlType.kDutyCycle);
    }
  }

  @Override
  public void startLauncher(double new_Setpoint) {
    currentLauncherSetpoint = new_Setpoint;
    launchController.setReference(new_Setpoint, ControlType.kDutyCycle);
    launcherIsEnabled = true;
  }

  @Override
  public double getRPMLeft() {
    return leftEncoder.getVelocity();
  }

  @Override
  public double getRPMRight() {
    return rightEncoder.getVelocity();
  }

  @Override
  public double getRPMLauncher() {
    return launchEncoder.getVelocity();
  }

  public double getSetpointLauncher() {
    return currentLauncherSetpoint;
  }

  @Override
  public boolean isLoaded() {
    return !loadedSensor.get();
  }

  @Override
  public void setPIDLeft(double newkP, double newkI, double newkD) {
    leftConfig.closedLoop.p(newkP).i(newkI).d(newkD);
    leftMotor.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    printf("New PID Left: %f, %f, %f", newkP, newkI, newkD);
  }

  @Override
  public void setPIDRight(double newkP, double newkI, double newkD) {
    rightConfig.closedLoop.p(newkP).i(newkI).d(newkD);
    rightMotor.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    printf("New PID Right: %f, %f, %f", newkP, newkI, newkD);
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
