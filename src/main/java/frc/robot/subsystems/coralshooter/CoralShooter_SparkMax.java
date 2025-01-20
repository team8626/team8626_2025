package frc.robot.subsystems.coralshooter;

import static frc.robot.subsystems.coralshooter.CoralShooterConstants.flywheelConfig;
import static frc.robot.subsystems.coralshooter.CoralShooterConstants.launcherConfig;

import org.ejml.dense.row.factory.LinearSolverFactory_MT_DDRM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.CS_InterfaceBase;

public class CoralShooter_SparkMax implements CoralShooterInterface, CS_InterfaceBase {
  
  private final CoralShooterValues values = new CoralShooterValues();
  
  // Declare the subsystem specific hardware here
  // Example:
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final SparkMax launchMotor;

  private final SparkMaxConfig leftConfig;
  private final SparkMaxConfig rightConfig;
  private final SparkMaxConfig launchConfig;

  private final SparkClosedLoopController leftController;
  private final RelativeEncoder leftEncoder;
  private final SparkClosedLoopController launchController;

  private boolean shooterIsEnabled = false;
  private boolean launcherIsEnabled = false;

  public CoralShooter_SparkMax() {
    // Setup configuration for the left motor
    leftConfig = new SparkMaxConfig();
    leftConfig.inverted(false);
    
    leftConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    leftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control.
        .p(CoralShooterConstants.gains.kP())
        .i(CoralShooterConstants.gains.kI())
        .d(CoralShooterConstants.gains.kD())
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
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    // Launcher Motor
    launchConfig = new SparkMaxConfig();
    launchConfig.inverted(false);

    launchMotor = new SparkMax(launcherConfig.leftCANID(), MotorType.kBrushless);
    launchMotor.configure(launchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    launchController = launchMotor.getClosedLoopController();
    launchController.setReference(0, ControlType.kDutyCycle);
  }

  @Override
  public void updateInputs(CoralShooterValues values) {
    values.launchIsEnabled = launcherIsEnabled;
    values.shooterIsEnabled = shooterIsEnabled;
    values.currentRPMLeft = getShooterRPMLeft();
    values.currentRPMRight = getShooterRPMRight();
    values.currentLauncherSpeed = launchMotor.getAppliedOutput();
  }

  @Override
  public void stopShooter() {
    setShooterRPM(0);
    shooterIsEnabled = false;

  }

  @Override
  public void stopLauncher() {
    setLauncherSpeed(0);
    launcherIsEnabled = false;
  }

  @Override
  public void setLauncherSpeed(double new_speed) {
    launchController.setReference(new_speed, ControlType.kDutyCycle);
    launcherIsEnabled = true; // (new_speed != 0);
  }

  @Override
  public void setShooterRPM(double new_RPM) {
    leftController.setReference(new_RPM, ControlType.kVelocity);
    shooterIsEnabled = true; // (new_speed != 0);
  }

  @Override
  public void setPID(double newkP, double newkI, double newkD) {
    leftConfig.closedLoop
        .p(newkP)
        .i(newkI)
        .d(newkD);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightConfig.closedLoop
        .p(newkP)
        .i(newkI)
        .d(newkD);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    printf("New PID: %f, %f, %f \n", newkP, newkI, newkD);
  }


  @Override
  public double getShooterRPMLeft() {
    return leftEncoder.getVelocity();
  }

  @Override
  public double getShooterRPMRight() {
    return leftEncoder.getVelocity();
  }
}
