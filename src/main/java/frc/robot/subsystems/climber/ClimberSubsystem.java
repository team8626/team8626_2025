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
    climberInterface.startClimber(shootingRPM);
  }

  public void setClimberRPM(double new_RPM) {
    shootingRPM = new_RPM;
    climberInterface.updateClimberRPM(new_RPM);
  }

  public void startIntake() {
    climberInterface.startClimber(ClimberConstants.intakeRPM);
  }

  public void stopClimber() {
    climberInterface.stopClimber();
  }

  public void stopAll() {
    climberInterface.stopClimber();
  }

  public double getClimberRPM() {
    return climberInterface.getClimberRPM();
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
    SmartDashboard.putBoolean("Subsystem/Climber/Climber", values.climberIsEnabled);

    SmartDashboard.putNumber("Subsystem/Climber/Chooter RPM", values.currentRPM);

    SmartDashboard.putNumber("Subsystem/Climber/Climber Amps", values.amps);

    double newRPM =
        SmartDashboard.getNumber("Subsystem/Climber/Shooting RPM", ClimberConstants.shootRPM);
    if (newRPM != shootingRPM) {
      setClimberRPM(newRPM);
    }
    SmartDashboard.putNumber("Subsystem/Climber/Shooting RPM", shootingRPM);
  }

  // Characterization methods
  public void runCharacterization(double input) {
    climberInterface.runCharacterization(input);
  }

  public double getCharacterizationVelocity() {
    return (values.currentRPM);
  }
}
