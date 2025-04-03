package frc.robot.subsystems.coralshooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.coralshooter.CoralShooterConstants.flywheelConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.CS_InterfaceBase;

public class CoralShooter_Sim implements CoralShooterInterface, CS_InterfaceBase {

  private boolean shooterIsEnabled = false;
  private boolean launcherIsEnabled = false;

  private double currentLauncherSetpoint = 0;

  private FlywheelSim leftSim;
  private FlywheelSim rightSim;
  private FlywheelSim launchSim;

  private DigitalInput loadedSensor = new DigitalInput(CoralShooterConstants.lidarPortBottom);
  private DigitalInput coralSensor1 = new DigitalInput(CoralShooterConstants.lidarPort1Top);
  private DigitalInput coralSensor2 = new DigitalInput(CoralShooterConstants.lidarPort2Top);
  private DIOSim loadedSensorSim = new DIOSim(loadedSensor);
  private DIOSim coralSensorSim1 = new DIOSim(coralSensor1);
  private DIOSim coralSensorSim2 = new DIOSim(coralSensor2);

  public CoralShooter_Sim() {

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
  public void updateInputs(CoralShooterValues values) {
    rightSim.update(0.02);
    leftSim.update(0.02);
    launchSim.update(0.02);

    values.launchIsEnabled = launcherIsEnabled;
    values.shooterIsEnabled = shooterIsEnabled;

    values.currentRPMLeft = getRPMLeft();
    values.currentRPMRight = getRPMRight();

    values.currentRMPLauncher = getRPMLauncher();
    values.currentLauncherSetpoint = getSetpointLauncher();

    values.ampsLeft = Amps.of(leftSim.getCurrentDrawAmps());
    values.ampsRight = Amps.of(rightSim.getCurrentDrawAmps());
    values.ampsLauncher = Amps.of(launchSim.getCurrentDrawAmps());

    values.isLoaded = isLoaded();
  }

  @Override
  public void startShooter(AngularVelocity new_RPMLeft, AngularVelocity new_RPMRight) {
    leftSim.setAngularVelocity(new_RPMLeft.in(RadiansPerSecond));
    rightSim.setAngularVelocity(new_RPMRight.in(RadiansPerSecond));
    shooterIsEnabled = true;
    printf("Shooter RPM: %f / %f", new_RPMLeft.in(RPM), new_RPMRight.in(RPM));
  }

  @Override
  public void stopShooter() {
    updateRPMShooter(RPM.of(0), RPM.of(0));
    shooterIsEnabled = false;
  }

  @Override
  public void updateRPMShooter(AngularVelocity new_RPMLeft, AngularVelocity new_RPMRight) {
    if (shooterIsEnabled) {
      leftSim.setAngularVelocity(new_RPMLeft.in(RadiansPerSecond));
      rightSim.setAngularVelocity(new_RPMRight.in(RadiansPerSecond));
      printf("Shooter RPM: %f / %f", new_RPMLeft.in(RPM), new_RPMRight.in(RPM));
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
  public AngularVelocity getRPMLeft() {
    return RPM.of(leftSim.getAngularVelocityRPM());
  }

  @Override
  public AngularVelocity getRPMRight() {
    return RPM.of(rightSim.getAngularVelocityRPM());
  }

  @Override
  public AngularVelocity getRPMLauncher() {
    return RPM.of(launchSim.getAngularVelocityRPM());
  }

  public double getSetpointLauncher() {
    return currentLauncherSetpoint;
  }

  @Override
  public boolean isLoaded() {
    return !loadedSensorSim.getValue();
  }

  @Override
  public boolean hasCoral() {
    return !coralSensorSim1.getValue() || !coralSensorSim2.getValue();
  }

  @Override
  public void setPIDLeft(double newkP, double newkI, double newkD) {
    printf("New PID Left: %f, %f, %f \n", newkP, newkI, newkD);
  }

  @Override
  public void setPIDRight(double newkP, double newkI, double newkD) {
    printf("New PID Right: %f, %f, %f \n", newkP, newkI, newkD);
  }
}
