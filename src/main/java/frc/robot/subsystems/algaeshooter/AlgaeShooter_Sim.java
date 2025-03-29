package frc.robot.subsystems.algaeshooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.algaeshooter.AlgaeShooterConstants.flywheelConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
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

    values.ampsLeft = Amps.of(leftSim.getCurrentDrawAmps());
    values.ampsRight = Amps.of(rightSim.getCurrentDrawAmps());
    values.ampsLauncher = Amps.of(launchSim.getCurrentDrawAmps());

    values.isLoaded = shooterIsLoaded();
  }

  @Override
  public void startShooter(AngularVelocity new_RPM) {
    rightSim.setAngularVelocity(new_RPM.in(RadiansPerSecond));
    leftSim.setAngularVelocity(new_RPM.in(RadiansPerSecond));
    shooterIsEnabled = true;
    printf("Shooter RPM: %f", new_RPM.in(RPM));
  }

  @Override
  public void stopShooter() {
    updateShooterRPM(RPM.of(0));
    shooterIsEnabled = false;
  }

  @Override
  public void updateShooterRPM(AngularVelocity new_RPM) {
    if (shooterIsEnabled) {
      rightSim.setAngularVelocity(new_RPM.in(RotationsPerSecond));
      leftSim.setAngularVelocity(new_RPM.in(RotationsPerSecond));
      printf("Shooter RPM: %f", new_RPM.in(RPM));
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
  public AngularVelocity getShooterRPMLeft() {
    return RPM.of(leftSim.getAngularVelocityRPM());
  }

  @Override
  public AngularVelocity getShooterRPMRight() {
    return RPM.of(rightSim.getAngularVelocityRPM());
  }

  @Override
  public AngularVelocity getLauncherRPM() {
    return RPM.of(launchSim.getAngularVelocityRPM());
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
