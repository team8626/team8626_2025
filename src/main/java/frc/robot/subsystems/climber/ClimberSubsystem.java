package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.climber.ClimberInterface.ClimberValues;
import frc.utils.CS_Utils;

public class ClimberSubsystem extends CS_SubsystemBase {
  private ClimberInterface climberInterface;
  private ClimberValues values;
  private double desiredAngleDegrees = ClimberConstants.restAngleDegrees;

  public ClimberSubsystem(ClimberInterface subsystem_interface) {
    super();

    this.climberInterface = subsystem_interface;
    values = new ClimberValues();
    println("Created");
  }

  public void setAngleDegrees(double new_angle) {
    printf("Setting Angle to: %f deg", new_angle);
    desiredAngleDegrees = new_angle;
    climberInterface.setAngleDegrees(new_angle);
  }

  // public void stop() {
  //   climberInterface.stop();
  // }

  public double getAngleDegrees() {
    return climberInterface.getAngleDegrees();
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
  }

  @Override
  public void updateDashboard() {
    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    double newkP = SmartDashboard.getNumber("Subsystem/Climber/Gains/P", values.kP);
    double newkI = SmartDashboard.getNumber("Subsystem/Climber/Gains/I", values.kI);
    double newkD = SmartDashboard.getNumber("Subsystem/Climber/Gains/D", values.kD);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kP = CS_Utils.updateFromSmartDashboard(newkP, values.kP, (value) -> setkP(value));
    values.kI = CS_Utils.updateFromSmartDashboard(newkI, values.kI, (value) -> setkI(value));
    values.kD = CS_Utils.updateFromSmartDashboard(newkD, values.kD, (value) -> setkD(value));

    // Update the SmartDashboard with the current state of the subsystem
    SmartDashboard.putBoolean("Subsystem/Climber/IsEnabled", values.climberIsEnabled);
    SmartDashboard.putNumber("Subsystem/Climber/CurrentAngle", values.currentAngleDegrees);
    SmartDashboard.putNumber("Subsystem/Climber/Amps", values.amps);

    // double newAngle =
    //     SmartDashboard.getNumber(
    //         "Subsystem/Climber/DesiredAngle", ClimberConstants.restAngleDegrees);
    // if (newAngle != desiredAngleDegrees) {
    //   setAngleDegrees(newAngle);
    // }
    SmartDashboard.putNumber("Subsystem/Climber/DesiredAngle", desiredAngleDegrees);
  }

  // Characterization methods
  public void runCharacterization(double input) {
    climberInterface.runCharacterization(input);
  }

  public double getCharacterizationVelocity() {
    return (values.currentRPM);
  }
}
