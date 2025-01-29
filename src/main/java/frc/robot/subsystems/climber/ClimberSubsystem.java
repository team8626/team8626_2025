package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.climber.ClimberInterface.ClimberValues;
import frc.utils.CS_Utils;

public class ClimberSubsystem extends CS_SubsystemBase {
  private ClimberInterface climberInterface;
  private ClimberValues values;
  private double shootingRPM = ClimberConstants.shootRPM;

  public ClimberSubsystem(ClimberInterface subsystem_interface) {
    super();

    this.climberInterface = subsystem_interface;
    values = new ClimberValues();
    println("Created");
  }

  // Calls to the Climber interface
  public void startRampUp() {
    climberInterface.startShooter(shootingRPM);
  }

  public void setShooterRPM(double new_RPM) {
    shootingRPM = new_RPM;
    climberInterface.updateShooterRPM(new_RPM);
  }

  public void startIntake() {
    climberInterface.startShooter(ClimberConstants.intakeRPM);
    climberInterface.startLauncher(ClimberConstants.launcherIntakeSetpoint);
  }

  public void startLauncher(double new_Setpoint) {
    climberInterface.startLauncher(new_Setpoint);
  }

  public void stopShooter() {
    climberInterface.stopShooter();
  }

  public void stopLauncher() {
    climberInterface.stopLauncher();
  }

  public void stopAll() {
    climberInterface.stopShooter();
    climberInterface.stopLauncher();
  }

  public double getShooterRPMLeft() {
    return climberInterface.getShooterRPMLeft();
  }

  public double getShooterRPMRight() {
    return climberInterface.getShooterRPMRight();
  }

  public boolean isLoaded() {
    return climberInterface.shooterIsLoaded();
  }

  public void setPID(double newkP, double newkI, double newkD) {
    climberInterface.setPID(newkP, newkI, newkD);
  }

  public void setkP(double newkP) {
    climberInterface.setPID(newkP, values.kI, values.kD);
  }

  public void setkI(double newkI) {
    climberInterface.setPID(values.kP, newkI, values.kD);
  }

  public void setkD(double newkD) {
    climberInterface.setPID(values.kP, values.kI, newkD);
  }

  @Override
  public void CS_periodic() {
    climberInterface.updateInputs(values);
  }

  @Override
  public void initDashboard() {
    println("Initializing Dashboard");

    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    SmartDashboard.putNumber("Subsystem/Climber/Gains/P", ClimberConstants.gains.kP());
    SmartDashboard.putNumber("Subsystem/Climber/Gains/I", ClimberConstants.gains.kI());
    SmartDashboard.putNumber("Subsystem/Climber/Gains/D", ClimberConstants.gains.kD());

    SmartDashboard.putNumber("Subsystem/Climber/Last Shot in (ms)", 0);
  }

  @Override
  public void updateDashboard() {
    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    double newkP = SmartDashboard.getNumber("Subsystem/Climber/Gains/P", values.kP);
    double newkI = SmartDashboard.getNumber("Subsystem/Climber/Gains/I", values.kI);
    double newkD = SmartDashboard.getNumber("Subsystem/Climber/Gains/D", values.kD);
    // double newFF = SmartDashboard.getNumber("Subsystem/Climber/FF", values.FF);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kP = CS_Utils.updateFromSmartDashboard(newkP, values.kP, (value) -> setkP(value));
    values.kI = CS_Utils.updateFromSmartDashboard(newkI, values.kI, (value) -> setkI(value));
    values.kD = CS_Utils.updateFromSmartDashboard(newkD, values.kD, (value) -> setkD(value));

    // Update the SmartDashboard with the current state of the subsystem
    SmartDashboard.putBoolean("Subsystem/Climber/Shooter", values.shooterIsEnabled);
    SmartDashboard.putBoolean("Subsystem/Climber/Launcher", values.launchIsEnabled);

    SmartDashboard.putNumber("Subsystem/Climber/Shooter RPM Left", values.currentRPMLeft);
    SmartDashboard.putNumber("Subsystem/Climber/Shooter RPM Right", values.currentRPMRight);
    SmartDashboard.putNumber(
        "Subsystem/Climber/launcher RPM Right", values.currentRMPLauncher);
    SmartDashboard.putNumber(
        "Subsystem/Climber/Launcher Setpoint", values.currentLauncherSetpoint);

    SmartDashboard.putNumber("Subsystem/Climber/Shooter Amps Left", values.ampsLeft);
    SmartDashboard.putNumber("Subsystem/Climber/Shooter Amps Right", values.ampsRight);
    SmartDashboard.putNumber("Subsystem/Climber/Launcher Amps", values.ampsLauncher);

    SmartDashboard.putBoolean("Subsystem/Climber/isLoaded", values.isLoaded);

    double newRPM =
        SmartDashboard.getNumber("Subsystem/Climber/Shooting RPM", ClimberConstants.shootRPM);
    if (newRPM != shootingRPM) {
      setShooterRPM(newRPM);
    }
    SmartDashboard.putNumber("Subsystem/Climber/Shooting RPM", shootingRPM);
  }

  // Characterization methods
  public void runCharacterization(double input) {
    climberInterface.runCharacterizationLeft(input);
    climberInterface.runCharacterizationRight(input);
  }

  public double getCharacterizationVelocity() {
    return (values.currentRPMLeft + values.currentRPMRight) / 2.0;
  }
}
