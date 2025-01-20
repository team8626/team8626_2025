package frc.robot.subsystems.coralshooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.CS_InterfaceBase;
import static frc.robot.subsystems.coralshooter.CoralShooterConstants.flywheelConfig;

public class CoralShooter_Sim implements CoralShooterInterface, CS_InterfaceBase {

  private final CoralShooterValues values = new CoralShooterValues();

  private boolean shooterIsEnabled = false;
  private boolean launcherIsEnabled = false;

  private FlywheelSim leftSim;
  private FlywheelSim rightSim;
  private FlywheelSim launchSim;

  public CoralShooter_Sim() {
  
    leftSim = new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), flywheelConfig.momentOfInertia(), flywheelConfig.reduction()),
          DCMotor.getNEO(1),
          0.00363458292);
    rightSim = new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), flywheelConfig.momentOfInertia(), flywheelConfig.reduction()),
          DCMotor.getNEO(1),
          0.00363458292);
    launchSim = new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), flywheelConfig.momentOfInertia(), flywheelConfig.reduction()),
          DCMotor.getNEO(1),
          0.00363458292);
    }

  @Override
  public void updateInputs(CoralShooterValues values) {
    values.launchIsEnabled = launcherIsEnabled;
    values.shooterIsEnabled = shooterIsEnabled;
    values.currentRPMLeft = getShooterRPMLeft();
    values.currentRPMRight = getShooterRPMRight();
    values.currentLauncherSpeed = launchSim.getOutput(0);
  }

  @Override
  public void stopShooter() {
    setShooterRPM(0);
    shooterIsEnabled = false;
  }

  @Override
  public void setPID(double newkP, double newkI, double newkD) {
    printf("New PID: %f, %f, %f \n", newkP, newkI, newkD);
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
  public void setShooterRPM(double new_speed) {
    rightSim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(new_speed));
    leftSim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(new_speed));
    shooterIsEnabled = true;
  }

  @Override
  public void stopLauncher() {
    launchSim.setAngularVelocity(0);
    launcherIsEnabled = false;
  }

  @Override
  public void setLauncherSpeed(double new_speed) {
    launchSim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(new_speed));
    launcherIsEnabled = true;
  }
}
