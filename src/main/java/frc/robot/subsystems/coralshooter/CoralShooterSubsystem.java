package frc.robot.subsystems.coralshooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.coralshooter.CoralShooterInterface.CoralShooterValues;
import frc.utils.CS_Utils;

public class CoralShooterSubsystem extends CS_SubsystemBase {
  private CoralShooterInterface coralShooterInterface;
  private CoralShooterValues values;
  private double shootingRPMLeft = CoralShooterConstants.RPMShootLeft;
  private double shootingRPMRight = CoralShooterConstants.RPMShootRight;

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
        "Subsystem/CoralShooter/Gains/P_Left", CoralShooterConstants.gainsLeft.kP());
    SmartDashboard.putNumber(
        "Subsystem/CoralShooter/Gains/I_Left", CoralShooterConstants.gainsLeft.kI());
    SmartDashboard.putNumber(
        "Subsystem/CoralShooter/Gains/D_Left", CoralShooterConstants.gainsLeft.kD());
    SmartDashboard.putNumber(
        "Subsystem/CoralShooter/Gains/P_Right", CoralShooterConstants.gainsRight.kP());
    SmartDashboard.putNumber(
        "Subsystem/CoralShooter/Gains/I_Right", CoralShooterConstants.gainsRight.kI());
    SmartDashboard.putNumber(
        "Subsystem/CoralShooter/Gains/D_Right", CoralShooterConstants.gainsRight.kD());

    SmartDashboard.putNumber("Subsystem/CoralShooter/LastShotIn(ms)", 0);
  }

  @Override
  public void updateDashboard() {
    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    double newkPLeft =
        SmartDashboard.getNumber("Subsystem/CoralShooter/Gains/P_Left", values.kPLeft);
    double newkILeft =
        SmartDashboard.getNumber("Subsystem/CoralShooter/Gains/I_Left", values.kILeft);
    double newkDLeft =
        SmartDashboard.getNumber("Subsystem/CoralShooter/Gains/D_Left", values.kDLeft);
    double newkPRight =
        SmartDashboard.getNumber("Subsystem/CoralShooter/Gains/P_Left", values.kPRight);
    double newkIRight =
        SmartDashboard.getNumber("Subsystem/CoralShooter/Gains/I_Left", values.kIRight);
    double newkDRight =
        SmartDashboard.getNumber("Subsystem/CoralShooter/Gains/D_Left", values.kDRight);

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
            "Subsystem/CoralShooter/ShootingRPMRight", CoralShooterConstants.RPMShootRight);
    if (newRPMRight != shootingRPMRight) {
      setShooterRPM(shootingRPMLeft, newRPMRight);
    }
    SmartDashboard.putNumber("Subsystem/CoralShooter/ShootingRPMLeft", shootingRPMLeft);
    SmartDashboard.putNumber("Subsystem/CoralShooter/ShootingRPMRight", shootingRPMRight);
  }

  // Characterization methods
  public void runCharacterizationLeft(double input) {
    coralShooterInterface.runCharacterizationLeft(input);
  }

  public void runCharacterizationRight(double input) {
    coralShooterInterface.runCharacterizationRight(input);
  }

  public double getCharacterizationVelocityLeft() {
    return values.currentRPMLeft;
  }

  public double getCharacterizationVelocityRight() {
    return values.currentRPMRight;
  }
}
