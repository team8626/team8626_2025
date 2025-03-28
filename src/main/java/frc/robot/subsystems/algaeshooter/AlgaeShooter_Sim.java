package frc.robot.subsystems.algaeshooter;

import static frc.robot.subsystems.algaeshooter.AlgaeShooterConstants.flywheelConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.CS_InterfaceBase;

public class AlgaeShooter_Sim implements AlgaeShooterInterface, CS_InterfaceBase {

  private boolean shooterIsEnabled = false;
  private boolean launcherIsEnabled = false;

  private double currentLauncherSetpoint = 0;

  private FlywheelSim leftSim;
  private FlywheelSim rightSim;
  private FlywheelSim launchSim;

  private DigitalInput loadedSensor = new DigitalInput(AlgaeShooterConstants.infraRedPort);
  private DIOSim loadedSensorSim = new DIOSim(loadedSensor);

  public AlgaeShooter_Sim() {

    leftSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1),
                4 * flywheelConfig.momentOfInertia(),
                flywheelConfig.reduction()),
            DCMotor.getNEO(1),
            0.00363458292);
    rightSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1),
                4 * flywheelConfig.momentOfInertia(),
                flywheelConfig.reduction()),
            DCMotor.getNEO(1),
            0.00363458292);
    launchSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), flywheelConfig.momentOfInertia(), flywheelConfig.reduction()),
            DCMotor.getNEO(1),
            0.00363458292);
  }

  @Override
  public void updateInputs(
      AlgaeShooterValues values) { // what is going on here does this need to be Algae
    rightSim.update(0.02);
    leftSim.update(0.02);
    launchSim.update(0.02);

    values.launchIsEnabled = launcherIsEnabled;
    values.shooterIsEnabled = shooterIsEnabled;

    values.currentRPMLeft = getShooterRPMLeft();
    values.currentRPMRight = getShooterRPMRight();

    values.currentRMPLauncher = getLauncherRPM();
    values.currentLauncherSetpoint = getLauncherSetpoint();

    values.ampsLeft = leftSim.getCurrentDrawAmps();
    values.ampsRight = rightSim.getCurrentDrawAmps();
    values.ampsLauncher = launchSim.getCurrentDrawAmps();

    values.isLoaded = shooterIsLoaded();
  }

  @Override
  public void startShooter(double new_RPM) {
    rightSim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(new_RPM));
    leftSim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(new_RPM));
    shooterIsEnabled = true;
    printf("Shooter RPM: %f", new_RPM);
  }

  @Override
  public void stopShooter() {
    updateShooterRPM(0);
    shooterIsEnabled = false;
  }

  @Override
  public void updateShooterRPM(double new_RPM) {
    if (shooterIsEnabled) {
      rightSim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(new_RPM));
      leftSim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(new_RPM));
      printf("Shooter RPM: %f", new_RPM);
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
      launchSim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(new_Setpoint));
    }
    printf("Launcher Setpoint: %f", new_Setpoint);
  }

  @Override
  public void startLauncher(double new_Setpoint) {
    currentLauncherSetpoint = new_Setpoint;
    launcherIsEnabled = true;
    printf("Launcher Setpoint: %f", new_Setpoint);
  }

  @Override
  public void startShooterBySetpoint(double new_Setpoint) {
    // rightController.setReference(new_Setpoint, ControlType.kDutyCycle);
    // leftController.setReference(new_Setpoint, ControlType.kDutyCycle);
    shooterIsEnabled = true;
    printf("Motors Setpoint: %f", new_Setpoint);
  }

  @Override
  public double getShooterRPMLeft() {
    return leftSim.getAngularVelocityRPM();
  }

  @Override
  public double getShooterRPMRight() {
    return rightSim.getAngularVelocityRPM();
  }

  @Override
  public double getLauncherRPM() {
    return launchSim.getAngularVelocityRPM();
  }

  public double getLauncherSetpoint() {
    return currentLauncherSetpoint;
  }

  @Override
  public boolean shooterIsLoaded() {
    return !loadedSensorSim.getValue();
  }

  @Override
  public void setPID(double newkP, double newkI, double newkD) {
    printf("New PID: %f, %f, %f \n", newkP, newkI, newkD);
  }
}
