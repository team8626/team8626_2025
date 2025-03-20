package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.wrist.WristInterface.WristValues;
import frc.utils.CS_Utils;

public class WristSubsystem extends CS_SubsystemBase {
  private WristInterface wristInterface;
  private WristValues values;

  public WristSubsystem(WristInterface subsystem_interface) {
    super();

    this.wristInterface = subsystem_interface;
    values = new WristValues();
    println("Created");
  }

  // Calls to the wrist interface

  public void setAngleDegrees(double new_Angle) {
    println("Setting angle to " + new_Angle);
    wristInterface.setAngleDegrees(new_Angle);
  }

  public double getAngleDegrees() {
    return values.currentAngleDegrees;
  }

  public void setPID(double newkP, double newkI, double newkD) {
    wristInterface.setPID(newkP, newkI, newkD);
  }

  public void setkP(double newkP) {
    wristInterface.setPID(newkP, values.kI, values.kD);
  }

  public void setkI(double newkI) {
    wristInterface.setPID(values.kP, newkI, values.kD);
  }

  public void setkD(double newkD) {
    wristInterface.setPID(values.kP, values.kI, newkD);
  }

  public void goUp(double offsetDegrees) {
    wristInterface.goUp(offsetDegrees);
  }

  public void goDown(double offsetDegrees) {
    wristInterface.goDown(offsetDegrees);
  }

  @Override
  public void CS_periodic() {
    wristInterface.updateInputs(values);
  }

  @Override
  public void initDashboard() {
    println("Initializing Dashboard");

    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    SmartDashboard.putNumber("Subsystem/Wrist/Gains/P", WristConstants.gains.kP());
    SmartDashboard.putNumber("Subsystem/Wrist/Gains/I", WristConstants.gains.kI());
    SmartDashboard.putNumber("Subsystem/Wrist/Gains/D", WristConstants.gains.kD());
  }

  @Override
  public void updateDashboard() {
    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    double newkP = SmartDashboard.getNumber("Subsystem/Wrist/Gains/P", values.kP);
    double newkI = SmartDashboard.getNumber("Subsystem/Wrist/Gains/I", values.kI);
    double newkD = SmartDashboard.getNumber("Subsystem/Wrist/Gains/D", values.kD);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kP = CS_Utils.updateFromSmartDashboard(newkP, values.kP, (value) -> setkP(value));
    values.kI = CS_Utils.updateFromSmartDashboard(newkI, values.kI, (value) -> setkI(value));
    values.kD = CS_Utils.updateFromSmartDashboard(newkD, values.kD, (value) -> setkD(value));

    // Update the SmartDashboard with the current state of the subsystem
    SmartDashboard.putBoolean("Subsystem/Wrist/Enabled", values.isEnabled);
    SmartDashboard.putNumber("Subsystem/Wrist/CurrentAngleDegrees", values.currentAngleDegrees);
    SmartDashboard.putNumber("Subsystem/Wrist/Amps", values.amps);
    SmartDashboard.putNumber("Subsystem/Wrist/DesiredAngleDegrees", values.desiredAngleDegrees);
    SmartDashboard.putNumber("Subsystem/Wrist/AppliedOutput", values.appliedOutput);
    SmartDashboard.putNumber("Subsystem/Wrist/Temperature", values.temperature);

    SmartDashboard.putData(this);
    //   double newAngle =
    //       SmartDashboard.getNumber(
    //           "Subsystem/Wrist/desiredAngleDegrees", WristConstants.defaultAngleDegrees);
    //   if (newAngle != desiredAngleDegrees) {
    //     setAngleDegrees(newAngle);
    //   }
    //   SmartDashboard.putNumber("Subsystem/Wrist/desiredAngleDegrees", desiredAngleDegrees);
  }

  // Characterization methods
  public void runCharacterization(double input) {
    wristInterface.runCharacterization(input);
  }

  public double getCharacterizationAngle() {
    return values.currentAngleDegrees;
  }
}
