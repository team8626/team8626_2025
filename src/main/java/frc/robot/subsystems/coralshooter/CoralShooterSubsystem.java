package frc.robot.subsystems.coralshooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.coralshooter.CoralShooterInterface.CoralShooterValues;
import frc.utils.CS_Utils;

public class CoralShooterSubsystem extends CS_SubsystemBase {
  private CoralShooterInterface coralShooterInterface;
  private CoralShooterValues values;
  private double shootingRPMLeft = CoralShooterConstants.RPMShootLeft;
  private double shootingRPMRight = CoralShooterConstants.RPMShoolLeft;

  public CoralShooterSubsystem(CoralShooterInterface subsystem_interface) {
    super();

    this.coralShooterInterface = subsystem_interface;
    values = new CoralShooterValues();
    println("Created");
  }

  // Calls to the coralShooter interface
  public void startRampUp() {
    coralShooterInterface.startShooter(shootingRPMLeft, shootingRPMRight);
  }

  public void setShooterRPM(double new_RPMLeft, double new_RPMRight) {
    shootingRPMLeft = new_RPMLeft;
    shootingRPMRight = new_RPMRight;
    coralShooterInterface.updateRPMShooter(new_RPMLeft, new_RPMRight);
  }

  public void startIntake() {
    coralShooterInterface.startShooter(
        CoralShooterConstants.RPMIntake, CoralShooterConstants.RPMIntake);
    coralShooterInterface.startLauncher(CoralShooterConstants.launcherIntakeSetpoint);
  }

  public void startLauncher(double new_Setpoint) {
    coralShooterInterface.startLauncher(new_Setpoint);
  }

  public void stopShooter() {
    coralShooterInterface.stopShooter();
  }

  public void stopLauncher() {
    coralShooterInterface.stopLauncher();
  }

  public void stopAll() {
    coralShooterInterface.stopShooter();
    coralShooterInterface.stopLauncher();
  }

  public double getShooterRPMLeft() {
    return coralShooterInterface.getRPMLeft();
  }

  public double getShooterRPMRight() {
    return coralShooterInterface.getRPMRight();
  }

  public boolean isLoaded() {
    return coralShooterInterface.isLoaded();
  }

  public void setPID(double newkP, double newkI, double newkD) {
    coralShooterInterface.setPID(newkP, newkI, newkD);
  }

  public void setkP(double newkP) {
    coralShooterInterface.setPID(newkP, values.kI, values.kD);
  }

  public void setkI(double newkI) {
    coralShooterInterface.setPID(values.kP, newkI, values.kD);
  }

  public void setkD(double newkD) {
    coralShooterInterface.setPID(values.kP, values.kI, newkD);
  }

  @Override
  public void CS_periodic() {
    coralShooterInterface.updateInputs(values);
  }

  @Override
  public void initDashboard() {
    println("Initializing Dashboard");

    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    SmartDashboard.putNumber("Subsystem/CoralShooter/Gains/P", CoralShooterConstants.gains.kP());
    SmartDashboard.putNumber("Subsystem/CoralShooter/Gains/I", CoralShooterConstants.gains.kI());
    SmartDashboard.putNumber("Subsystem/CoralShooter/Gains/D", CoralShooterConstants.gains.kD());

    SmartDashboard.putNumber("Subsystem/CoralShooter/LastShotIn(ms)", 0);
  }

  @Override
  public void updateDashboard() {
    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    double newkP = SmartDashboard.getNumber("Subsystem/CoralShooter/Gains/P", values.kP);
    double newkI = SmartDashboard.getNumber("Subsystem/CoralShooter/Gains/I", values.kI);
    double newkD = SmartDashboard.getNumber("Subsystem/CoralShooter/Gains/D", values.kD);
    // double newFF = SmartDashboard.getNumber("Subsystem/CoralShooter/FF", values.FF);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kP = CS_Utils.updateFromSmartDashboard(newkP, values.kP, (value) -> setkP(value));
    values.kI = CS_Utils.updateFromSmartDashboard(newkI, values.kI, (value) -> setkI(value));
    values.kD = CS_Utils.updateFromSmartDashboard(newkD, values.kD, (value) -> setkD(value));

    // Update the SmartDashboard with the current state of the subsystem
    SmartDashboard.putBoolean("Subsystem/CoralShooter/Shooter", values.shooterIsEnabled);
    SmartDashboard.putBoolean("Subsystem/CoralShooter/Launcher", values.launchIsEnabled);

    SmartDashboard.putNumber("Subsystem/CoralShooter/CurrentRPMLeft", values.currentRPMLeft);
    SmartDashboard.putNumber("Subsystem/CoralShooter/CurrentRPMRight", values.currentRPMRight);
    SmartDashboard.putNumber(
        "Subsystem/CoralShooter/CurrentRPMLauncher", values.currentRMPLauncher);
    SmartDashboard.putNumber(
        "Subsystem/CoralShooter/LauncherSetpoint", values.currentLauncherSetpoint);

    SmartDashboard.putNumber("Subsystem/CoralShooter/ShooterAmpsLeft", values.ampsLeft);
    SmartDashboard.putNumber("Subsystem/CoralShooter/ShooterAmpsRight", values.ampsRight);
    SmartDashboard.putNumber("Subsystem/CoralShooter/LauncherAmps", values.ampsLauncher);

    SmartDashboard.putBoolean("Subsystem/CoralShooter/IsLoaded", values.isLoaded);

    double newRPMLeft =
        SmartDashboard.getNumber(
            "Subsystem/CoralShooter/ShootingRPMLeft", CoralShooterConstants.RPMShootLeft);
    if (newRPMLeft != shootingRPMLeft) {
      setShooterRPM(newRPMLeft, shootingRPMRight);
    }
    double newRPMRight =
        SmartDashboard.getNumber(
            "Subsystem/CoralShooter/ShootingRPMRight", CoralShooterConstants.RPMShoolLeft);
    if (newRPMRight != shootingRPMRight) {
      setShooterRPM(shootingRPMLeft, newRPMRight);
    }
    SmartDashboard.putNumber("Subsystem/CoralShooter/ShootingRPMLeft", shootingRPMLeft);
    SmartDashboard.putNumber("Subsystem/CoralShooter/ShootingRPMRight", shootingRPMRight);
  }

  // Characterization methods
  public void runCharacterization(double input) {
    coralShooterInterface.runCharacterizationLeft(input);
    coralShooterInterface.runCharacterizationRight(input);
  }

  public double getCharacterizationVelocity() {
    return (values.currentRPMLeft + values.currentRPMRight) / 2.0;
  }
}
