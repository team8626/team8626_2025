package frc.robot.subsystems.algaeshooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.algaeshooter.AlgaeShooterInterface.AlgaeShooterValues;
import frc.utils.CS_Utils;

public class AlgaeShooterSubsystem extends CS_SubsystemBase {
  private AlgaeShooterInterface algaeShooterInterface;
  private AlgaeShooterValues values;
  private AngularVelocity DesiredRPM;

  public AlgaeShooterSubsystem(AlgaeShooterInterface subsystem_interface) {
    super();

    this.algaeShooterInterface = subsystem_interface;
    values = new AlgaeShooterValues();
    println("Created");
  }

  // Calls to the algaeShooter interface
  public void startRampUp(AngularVelocity newRPM) {
    // algaeShooterInterface.startLauncher(AlgaeShooterConstants.launcherIntakeSetpoint);
    updateShooterRPM(newRPM);
    algaeShooterInterface.startShooter(DesiredRPM);
  }

  public void setShooterRPM(AngularVelocity newRPM) {
    updateShooterRPM(newRPM);
    algaeShooterInterface.updateShooterRPM(newRPM);
  }

  public void startIntake() {
    algaeShooterInterface.startShooter(AlgaeShooterConstants.intakeRPM);
    algaeShooterInterface.startLauncher(AlgaeShooterConstants.launcherIntakeSetpoint);
  }

  public void startDiscard() {
    algaeShooterInterface.startShooter(AlgaeShooterConstants.discardRPM);
    algaeShooterInterface.startLauncher(-AlgaeShooterConstants.launcherDiscardSetpoint);
  }

  public void startIntake(AngularVelocity newSpeed) {
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

  public AngularVelocity getShooterRPMLeft() {
    return algaeShooterInterface.getShooterRPMLeft();
  }

  public AngularVelocity getShooterRPMRight() {
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
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/Gains/P", AlgaeShooterConstants.gains.kP());
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/Gains/I", AlgaeShooterConstants.gains.kI());
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/Gains/D", AlgaeShooterConstants.gains.kD());

    SmartDashboard.putNumber("Subsystem/AlgaeShooterSubsystem/Last Shot in (ms)", 0);
    SmartDashboard.putBoolean("Commands/AlgaeShooterRampUp/OverrideRPM", false);
    SmartDashboard.putNumber("Commands/AlgaeShooterRampUp/ForcedRMP", 2200);
  }

  @Override
  public void updateDashboard() {
    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    double newkP = SmartDashboard.getNumber("Subsystem/AlgaeShooterSubsystem/Gains/P", values.kP);
    double newkI = SmartDashboard.getNumber("Subsystem/AlgaeShooterSubsystem/Gains/I", values.kI);
    double newkD = SmartDashboard.getNumber("Subsystem/AlgaeShooterSubsystem/Gains/D", values.kD);
    // double newFF = SmartDashboard.getNumber("Subsystem/AlgaeShooterSubsystem/FF", values.FF);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kP = CS_Utils.updateFromSmartDashboard(newkP, values.kP, (value) -> setkP(value));
    values.kI = CS_Utils.updateFromSmartDashboard(newkI, values.kI, (value) -> setkI(value));
    values.kD = CS_Utils.updateFromSmartDashboard(newkD, values.kD, (value) -> setkD(value));

    // Update the SmartDashboard with the current state of the subsystem
    SmartDashboard.putBoolean("Subsystem/AlgaeShooterSubsystem/Shooter", values.shooterIsEnabled);
    SmartDashboard.putBoolean("Subsystem/AlgaeShooterSubsystem/Launcher", values.launchIsEnabled);

    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/Shooter RPM Left", values.currentRPMLeft.in(RPM));
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/Shooter RPM Right", values.currentRPMRight.in(RPM));
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/launcher RPM Right", values.currentRMPLauncher.in(RPM));
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/Launcher Setpoint", values.currentLauncherSetpoint);

    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/ShooterAmpsLeft", values.ampsLeft.in(Amps));
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/ShooterAmpsRight", values.ampsRight.in(Amps));
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/LauncherAmps", values.ampsLauncher.in(Amps));
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/TemperatureShooterLeft", values.tempLeft.in(Celsius));
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/TemperatureShooterRight", values.tempRight.in(Celsius));
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/TemperatureLauncher", values.tempLauncher.in(Celsius));
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/AppliedOutputShooterLeft", values.appliedOutputLeft);
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/AppliedOutputShooterRight", values.appliedOutputRight);
    SmartDashboard.putNumber(
        "Subsystem/AlgaeShooterSubsystem/AppliedOutputLauncher", values.appliedOutputLauncher);

    SmartDashboard.putBoolean("Subsystem/AlgaeShooterSubsystem/isLoaded", values.isLoaded);

    AngularVelocity newRPM =
        RPM.of(
            SmartDashboard.getNumber(
                "Subsystem/AlgaeShooterSubsystem/Shooting RPM",
                AlgaeShooterConstants.shootRPM.in(RPM)));
    if (newRPM != DesiredRPM) {
      updateShooterRPM(newRPM);
      setShooterRPM(DesiredRPM);
    }

    SmartDashboard.putData(this);

    // SmartDashboard.putNumber("Subsystem/AlgaeShooterSubsystem/Shooting RPM", DesiredRPM);
  }

  // Characterization methods
  public void runCharacterization(double input) {
    algaeShooterInterface.runCharacterizationLeft(input);
    algaeShooterInterface.runCharacterizationRight(input);
  }

  public double getCharacterizationVelocity() {
    return (values.currentRPMLeft.in(RPM) + values.currentRPMRight.in(RPM)) / 2.0;
  }

  private void updateShooterRPM(AngularVelocity new_RPM) {
    DesiredRPM = new_RPM;
    SmartDashboard.putNumber("Subsystem/AlgaeShooterSubsystem/Shooting RPM", DesiredRPM.in(RPM));
  }
}
