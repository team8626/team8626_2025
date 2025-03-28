package frc.robot.subsystems.coralshooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.coralshooter.CoralShooterInterface.CoralShooterValues;
import frc.utils.CS_Utils;

public class CoralShooterSubsystem extends CS_SubsystemBase {
  private CoralShooterInterface coralShooterInterface;
  private CoralShooterValues values;
  private AngularVelocity shootingRPMLeft = RPM.of(0); // = CoralShooterConstants.RPMShootLeft;
  private AngularVelocity shootingRPMRight = RPM.of(0); // = CoralShooterConstants.RPMShootRight;

  public CoralShooterSubsystem(CoralShooterInterface subsystem_interface) {
    super();

    this.coralShooterInterface = subsystem_interface;
    values = new CoralShooterValues();
    println("Created");
  }

  public void startRampUp(AngularVelocity newRPMLeft, AngularVelocity newRPMRight) {
    shootingRPMLeft = newRPMLeft;
    shootingRPMRight = newRPMRight;
    coralShooterInterface.startShooter(shootingRPMLeft, shootingRPMRight);
  }

  public void setShooterRPM(AngularVelocity new_RPMLeft, AngularVelocity new_RPMRight) {
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

  public AngularVelocity getShooterRPMLeft() {
    return coralShooterInterface.getRPMLeft();
  }

  public AngularVelocity getShooterRPMRight() {
    return coralShooterInterface.getRPMRight();
  }

  public boolean isLoaded() {
    return coralShooterInterface.isLoaded();
  }

  public boolean hasCoral() {
    return coralShooterInterface.hasCoral();
  }

  public void setPIDLeft(double newkP, double newkI, double newkD) {
    coralShooterInterface.setPIDLeft(newkP, newkI, newkD);
  }

  public void setkPLeft(double newkP) {
    coralShooterInterface.setPIDLeft(newkP, values.kILeft, values.kDLeft);
  }

  public void setkILeft(double newkI) {
    coralShooterInterface.setPIDLeft(values.kPLeft, newkI, values.kDLeft);
  }

  public void setkDLeft(double newkD) {
    coralShooterInterface.setPIDLeft(values.kPLeft, values.kILeft, newkD);
  }

  public void setPIDRight(double newkP, double newkI, double newkD) {
    coralShooterInterface.setPIDRight(newkP, newkI, newkD);
  }

  public void setkPRight(double newkP) {
    coralShooterInterface.setPIDRight(newkP, values.kIRight, values.kDRight);
  }

  public void setkIRight(double newkI) {
    coralShooterInterface.setPIDRight(values.kPRight, newkI, values.kDRight);
  }

  public void setkDRight(double newkD) {
    coralShooterInterface.setPIDRight(values.kPRight, values.kIRight, newkD);
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
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/Gains/Left/P", CoralShooterConstants.gainsLeft.kP());
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/Gains/Left/I", CoralShooterConstants.gainsLeft.kI());
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/Gains/Left/D", CoralShooterConstants.gainsLeft.kD());
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/Gains/Right/P", CoralShooterConstants.gainsRight.kP());
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/Gains/Right/I", CoralShooterConstants.gainsRight.kI());
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/Gains/Right/D", CoralShooterConstants.gainsRight.kD());

    SmartDashboard.putNumber("Subsystem/CoralShooterSubsystem/LastShotIn(ms)", 0);

    SmartDashboard.putBoolean("Commands/CoralShooterRampUp/OverrideRPM", false);
    SmartDashboard.putNumber("Commands/CoralShooterRampUp/ForcedRMPLeft", 1275);
    SmartDashboard.putNumber("Commands/CoralShooterRampUp/ForcedRMPRight", 1275);
    SmartDashboard.putNumber(
        "Commands/ToPathAndFinkleAndCoralShootWithOffset/OffsetDistance(in)", 7.0);
    SmartDashboard.putNumber("Commands/DriveToPoseFinkle/OffsetDistance(in)", 7.0);

    SmartDashboard.putNumber("Subsystem/PresetManager/AimPreset/rmpErrorMultiplier", 1.0);
    SmartDashboard.putNumber("Subsystem/PresetManager/AimPreset/angleAdjust(deg)", 0);
  }

  @Override
  public void updateDashboard() {
    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    double newkPLeft =
        SmartDashboard.getNumber("Subsystem/CoralShooterSubsystem/Gains/Left/P", values.kPLeft);
    double newkILeft =
        SmartDashboard.getNumber("Subsystem/CoralShooterSubsystem/Gains/Left/I", values.kILeft);
    double newkDLeft =
        SmartDashboard.getNumber("Subsystem/CoralShooterSubsystem/Gains/Left/D", values.kDLeft);
    double newkPRight =
        SmartDashboard.getNumber("Subsystem/CoralShooterSubsystem/Gains/Right/P", values.kPRight);
    double newkIRight =
        SmartDashboard.getNumber("Subsystem/CoralShooterSubsystem/Gains/Right/I", values.kIRight);
    double newkDRight =
        SmartDashboard.getNumber("Subsystem/CoralShooterSubsystem/Gains/Right/D", values.kDRight);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kPLeft =
        CS_Utils.updateFromSmartDashboard(newkPLeft, values.kPLeft, (value) -> setkPLeft(value));
    values.kILeft =
        CS_Utils.updateFromSmartDashboard(newkILeft, values.kILeft, (value) -> setkILeft(value));
    values.kDLeft =
        CS_Utils.updateFromSmartDashboard(newkDLeft, values.kDLeft, (value) -> setkDLeft(value));
    values.kPRight =
        CS_Utils.updateFromSmartDashboard(newkPRight, values.kPRight, (value) -> setkPRight(value));
    values.kIRight =
        CS_Utils.updateFromSmartDashboard(newkIRight, values.kIRight, (value) -> setkIRight(value));
    values.kDRight =
        CS_Utils.updateFromSmartDashboard(newkDRight, values.kDRight, (value) -> setkDRight(value));

    // Update the SmartDashboard with the current state of the subsystem
    SmartDashboard.putBoolean("Subsystem/CoralShooterSubsystem/Shooter", values.shooterIsEnabled);
    SmartDashboard.putBoolean("Subsystem/CoralShooterSubsystem/Launcher", values.launchIsEnabled);

    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/CurrentRPMLeft", values.currentRPMLeft.in(RPM));
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/CurrentRPMRight", values.currentRPMRight.in(RPM));
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/CurrentRPMLauncher", values.currentRMPLauncher.in(RPM));
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/LauncherSetpoint", values.currentLauncherSetpoint);

    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/ShooterAmpsLeft", values.ampsLeft.in(Amps));
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/ShooterAmpsRight", values.ampsRight.in(Amps));
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/LauncherAmps", values.ampsLauncher.in(Amps));
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/TemperatureShooterLeft", values.tempLeft.in(Celsius));
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/TemperatureShooterRight", values.tempRight.in(Celsius));
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/TemperatureLauncher", values.tempLauncher.in(Celsius));
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/AppliedOutputShooterLeft", values.appliedOutputLeft);
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/AppliedOutputShooterRight", values.appliedOutputRight);
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/AppliedOutputLauncher", values.appliedOutputLauncher);
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/ShooterappliedOutputLeft", values.appliedOutputLeft);
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/ShooterappliedOutputRight", values.appliedOutputRight);
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/LauncherappliedOutputAmps", values.appliedOutputLauncher);

    SmartDashboard.putBoolean("Subsystem/CoralShooterSubsystem/IsLoaded", values.isLoaded);
    SmartDashboard.putBoolean(
        "Subsystem/CoralShooterSubsystem/CoralDetected", values.coralDetected);

    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/ShootingRPMLeft", shootingRPMLeft.in(RPM));
    SmartDashboard.putNumber(
        "Subsystem/CoralShooterSubsystem/ShootingRPMRight", shootingRPMRight.in(RPM));
  }

  // Characterization methods
  public void runCharacterizationLeft(double input) {
    coralShooterInterface.runCharacterizationLeft(input);
  }

  public void runCharacterizationRight(double input) {
    coralShooterInterface.runCharacterizationRight(input);
  }

  public double getCharacterizationVelocityLeft() {
    return values.currentRPMLeft.in(RPM);
  }

  public double getCharacterizationVelocityRight() {
    return values.currentRPMRight.in(RPM);
  }
}
