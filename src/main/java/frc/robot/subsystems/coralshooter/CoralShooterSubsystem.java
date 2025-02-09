package frc.robot.subsystems.coralshooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.coralshooter.CoralShooterInterface.CoralShooterValues;
import frc.utils.CS_Utils;

public class CoralShooterSubsystem extends CS_SubsystemBase {
  private CoralShooterInterface coralShooterInterface;
  private CoralShooterValues values;
  private double shootingRPM = CoralShooterConstants.shootRPM;

  public CoralShooterSubsystem(CoralShooterInterface subsystem_interface) {
    super();

    this.coralShooterInterface = subsystem_interface;
    values = new CoralShooterValues();
    println("Created");
  }

  // Calls to the coralShooter interface
  public void startRampUp() {
    coralShooterInterface.startShooter(shootingRPM);
  }

  public void setShooterRPM(double new_RPM) {
    shootingRPM = new_RPM;
    coralShooterInterface.updateShooterRPM(new_RPM);
  }

  public void startIntake() {
    coralShooterInterface.startShooter(CoralShooterConstants.intakeRPM);
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
    return coralShooterInterface.getShooterRPMLeft();
  }

  public double getShooterRPMRight() {
    return coralShooterInterface.getShooterRPMRight();
  }

  public boolean isLoaded() {
    return coralShooterInterface.shooterIsLoaded();
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

    SmartDashboard.putNumber("Subsystem/CoralShooter/Last Shot in (ms)", 0);
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

    SmartDashboard.putNumber("Subsystem/CoralShooter/Shooter RPM Left", values.currentRPMLeft);
    SmartDashboard.putNumber("Subsystem/CoralShooter/Shooter RPM Right", values.currentRPMRight);
    SmartDashboard.putNumber(
        "Subsystem/CoralShooter/launcher RPM Right", values.currentRMPLauncher);
    SmartDashboard.putNumber(
        "Subsystem/CoralShooter/Launcher Setpoint", values.currentLauncherSetpoint);

    SmartDashboard.putNumber("Subsystem/CoralShooter/Shooter Amps Left", values.ampsLeft);
    SmartDashboard.putNumber("Subsystem/CoralShooter/Shooter Amps Right", values.ampsRight);
    SmartDashboard.putNumber("Subsystem/CoralShooter/Launcher Amps", values.ampsLauncher);

    SmartDashboard.putBoolean("Subsystem/CoralShooter/isLoaded", values.isLoaded);

    double newRPM =
        SmartDashboard.getNumber(
            "Subsystem/CoralShooter/Shooting RPM", CoralShooterConstants.shootRPM);
    if (newRPM != shootingRPM) {
      setShooterRPM(newRPM);
    }
    SmartDashboard.putNumber("Subsystem/CoralShooter/Shooting RPM", shootingRPM);
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
