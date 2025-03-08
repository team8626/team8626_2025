package frc.robot.subsystems.algaeshooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.algaeshooter.AlgaeShooterInterface.AlgaeShooterValues;
import frc.utils.CS_Utils;

public class AlgaeShooterSubsystem extends CS_SubsystemBase {
  private AlgaeShooterInterface algaeShooterInterface;
  private AlgaeShooterValues values;
  private double shootingRPM = AlgaeShooterConstants.shootRPM;

  public AlgaeShooterSubsystem(AlgaeShooterInterface subsystem_interface) {
    super();

    this.algaeShooterInterface = subsystem_interface;
    values = new AlgaeShooterValues();
    println("Created");
  }

  // Calls to the algaeShooter interface
  public void startRampUp(double newRPM) {
    // algaeShooterInterface.startLauncher(AlgaeShooterConstants.launcherIntakeSetpoint);
    shootingRPM = newRPM;
    algaeShooterInterface.startShooter(shootingRPM);
  }

  public void setShooterRPM(double new_RPM) {
    shootingRPM = new_RPM;
    algaeShooterInterface.updateShooterRPM(new_RPM);
  }

  public void startIntake() {
    algaeShooterInterface.startShooter(AlgaeShooterConstants.intakeRPM);
    algaeShooterInterface.startLauncher(AlgaeShooterConstants.launcherIntakeSetpoint);
  }

  public void startIntake(double newSpeed) {
    algaeShooterInterface.startShooter(newSpeed);
    algaeShooterInterface.startLauncher(AlgaeShooterConstants.launcherIntakeSetpoint);
  }

  public void startLauncher(double new_Setpoint) {
    algaeShooterInterface.startLauncher(new_Setpoint);
  }

  public void startShooterBySetpoint(double new_Setpoint) {
    algaeShooterInterface.startShooterBySetpoint(new_Setpoint);
  }

  public void stopShooter() {
    algaeShooterInterface.stopShooter();
  }

  public void stopLauncher() {
    algaeShooterInterface.stopLauncher();
  }

  public void stopAll() {
    algaeShooterInterface.stopShooter();
    algaeShooterInterface.stopLauncher();
  }

  public double getShooterRPMLeft() {
    return algaeShooterInterface.getShooterRPMLeft();
  }

  public double getShooterRPMRight() {
    return algaeShooterInterface.getShooterRPMRight();
  }

  public boolean isLoaded() {
    return algaeShooterInterface.shooterIsLoaded();
  }

  public void setPID(double newkP, double newkI, double newkD) {
    algaeShooterInterface.setPID(newkP, newkI, newkD);
  }

  public void setkP(double newkP) {
    algaeShooterInterface.setPID(newkP, values.kI, values.kD);
  }

  public void setkI(double newkI) {
    algaeShooterInterface.setPID(values.kP, newkI, values.kD);
  }

  public void setkD(double newkD) {
    algaeShooterInterface.setPID(values.kP, values.kI, newkD);
  }

  @Override
  public void CS_periodic() {
    algaeShooterInterface.updateInputs(values);
  }

  @Override
  public void initDashboard() {
    println("Initializing Dashboard");

    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    SmartDashboard.putNumber("Subsystem/AlgaeShooter/Gains/P", AlgaeShooterConstants.gains.kP());
    SmartDashboard.putNumber("Subsystem/AlgaeShooter/Gains/I", AlgaeShooterConstants.gains.kI());
    SmartDashboard.putNumber("Subsystem/AlgaeShooter/Gains/D", AlgaeShooterConstants.gains.kD());

    SmartDashboard.putNumber("Subsystem/AlgaeShooter/Last Shot in (ms)", 0);
  }

  @Override
  public void updateDashboard() {
    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    double newkP = SmartDashboard.getNumber("Subsystem/AlgaeShooter/Gains/P", values.kP);
    double newkI = SmartDashboard.getNumber("Subsystem/AlgaeShooter/Gains/I", values.kI);
    double newkD = SmartDashboard.getNumber("Subsystem/AlgaeShooter/Gains/D", values.kD);
    // double newFF = SmartDashboard.getNumber("Subsystem/AlgaeShooter/FF", values.FF);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kP = CS_Utils.updateFromSmartDashboard(newkP, values.kP, (value) -> setkP(value));
    values.kI = CS_Utils.updateFromSmartDashboard(newkI, values.kI, (value) -> setkI(value));
    values.kD = CS_Utils.updateFromSmartDashboard(newkD, values.kD, (value) -> setkD(value));

    // Update the SmartDashboard with the current state of the subsystem
    SmartDashboard.putBoolean("Subsystem/AlgaeShooter/Shooter", values.shooterIsEnabled);
    SmartDashboard.putBoolean("Subsystem/AlgaeShooter/Launcher", values.launchIsEnabled);

    SmartDashboard.putNumber("Subsystem/AlgaeShooter/Shooter RPM Left", values.currentRPMLeft);
    SmartDashboard.putNumber("Subsystem/AlgaeShooter/Shooter RPM Right", values.currentRPMRight);
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooter/launcher RPM Right", values.currentRMPLauncher);
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooter/Launcher Setpoint", values.currentLauncherSetpoint);

    SmartDashboard.putNumber("Subsystem/AlgaeShooter/ShooterAmpsLeft", values.ampsLeft);
    SmartDashboard.putNumber("Subsystem/AlgaeShooter/ShooterAmpsRight", values.ampsRight);
    SmartDashboard.putNumber("Subsystem/AlgaeShooter/LauncherAmps", values.ampsLauncher);
    SmartDashboard.putNumber("Subsystem/AlgaeShooter/TemperatureShooterLeft", values.tempLeft);
    SmartDashboard.putNumber("Subsystem/AlgaeShooter/TemperatureShooterRight", values.tempRight);
    SmartDashboard.putNumber("Subsystem/AlgaeShooter/TemperatureLauncher", values.tempLauncher);
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooter/AppliedOutputShooterLeft", values.appliedOutputLeft);
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooter/AppliedOutputShooterRight", values.appliedOutputRight);
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooter/AppliedOutputLauncher", values.appliedOutputLauncher);

    SmartDashboard.putBoolean("Subsystem/AlgaeShooter/isLoaded", values.isLoaded);

    double newRPM =
        SmartDashboard.getNumber(
            "Subsystem/AlgaeShooter/Shooting RPM", AlgaeShooterConstants.shootRPM);
    if (newRPM != shootingRPM) {
      setShooterRPM(newRPM);
    }
    SmartDashboard.putNumber("Subsystem/AlgaeShooter/Shooting RPM", shootingRPM);
  }

  // Characterization methods
  public void runCharacterization(double input) {
    algaeShooterInterface.runCharacterizationLeft(input);
    algaeShooterInterface.runCharacterizationRight(input);
  }

  public double getCharacterizationVelocity() {
    return (values.currentRPMLeft + values.currentRPMRight) / 2.0;
  }
}
