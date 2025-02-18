package frc.robot.subsystems.algaeshooter;

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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.CS_InterfaceBase;

public class AlgaeShooter_SparkMax implements AlgaeShooterInterface, CS_InterfaceBase {

  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final SparkMax launchMotor;

  private final SparkMaxConfig leftConfig;
  private final SparkMaxConfig rightConfig;
  private final SparkMaxConfig launchConfig;

  private final SparkClosedLoopController leftController;
  private final RelativeEncoder leftEncoder;
  private final SparkClosedLoopController launchController;

  SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(gains.kS(), gains.kV(), gains.kA());

  private DigitalInput loadedSensor = new DigitalInput(AlgaeShooterConstants.infraRedPort);

  private boolean shooterIsEnabled = false;
  private boolean launcherIsEnabled = false;
  private double currentLauncherSetpoint = 0;

  public AlgaeShooter_SparkMax() {
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
        .p(AlgaeShooterConstants.gains.kP())
        .i(AlgaeShooterConstants.gains.kI())
        .d(AlgaeShooterConstants.gains.kD())
        // .velocityFF(0.002)
        .outputRange(-1, 1);

    leftMotor = new SparkMax(flywheelConfig.leftCANID(), MotorType.kBrushless);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftController = leftMotor.getClosedLoopController();
    leftEncoder = leftMotor.getEncoder();
    leftController.setReference(0, ControlType.kDutyCycle);

    // Setup configuration for the right motor (follower)
    rightConfig = new SparkMaxConfig();
    rightConfig.follow(leftMotor, true);

    rightMotor = new SparkMax(flywheelConfig.rightCANID(), MotorType.kBrushless);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Launcher Motor
    launchConfig = new SparkMaxConfig();
    launchConfig.inverted(false);

    launchMotor = new SparkMax(launcherConfig.leftCANID(), MotorType.kBrushless);
    launchMotor.configure(
        launchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    launchController = launchMotor.getClosedLoopController();
    launchController.setReference(0, ControlType.kDutyCycle);
  }

  @Override
  public void updateInputs(AlgaeShooterValues values) {
    values.launchIsEnabled = launcherIsEnabled;
    values.shooterIsEnabled = shooterIsEnabled;

    values.currentRPMLeft = getShooterRPMLeft();
    values.currentRPMRight = getShooterRPMRight();

    values.currentRMPLauncher = getLauncherRPM();
    values.currentLauncherSetpoint = getLauncherSetpoint();

    values.ampsLeft = leftMotor.getOutputCurrent();
    values.ampsRight = rightMotor.getOutputCurrent();
    values.ampsLauncher = launchMotor.getOutputCurrent();

    values.isLoaded = shooterIsLoaded();
  }

  @Override
  public void startShooter(double new_RPM) {
    leftController.setReference(
        new_RPM,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        leftFF.calculate(new_RPM),
        ArbFFUnits.kVoltage);

    shooterIsEnabled = true;
  }

  @Override
  public void stopShooter() {
    leftController.setReference(0, ControlType.kDutyCycle);
    shooterIsEnabled = false;
  }

  @Override
  public void updateShooterRPM(double new_RPM) {
    if (shooterIsEnabled) {
      leftController.setReference(
          new_RPM,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          leftFF.calculate(new_RPM),
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
    launchController.setReference(new_Setpoint, ControlType.kDutyCycle);
    launcherIsEnabled = true;
  }

  @Override
  public double getShooterRPMLeft() {
    return leftEncoder.getVelocity();
  }

  @Override
  public double getShooterRPMRight() {
    return leftEncoder.getVelocity();
  }

  @Override
  public double getLauncherRPM() {
    return leftEncoder.getVelocity();
  }

  public double getLauncherSetpoint() {
    return currentLauncherSetpoint;
  }

  @Override
  public boolean shooterIsLoaded() {
    return !loadedSensor.get();
  }

  @Override
  public void setPID(double newkP, double newkI, double newkD) {
    leftConfig.closedLoop.p(newkP).i(newkI).d(newkD);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightConfig.closedLoop.p(newkP).i(newkI).d(newkD);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    printf("New PID: %f, %f, %f", newkP, newkI, newkD);
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
