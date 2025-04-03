package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
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
  }

  // Calls to the wrist interface

  public void setAngle(Angle new_Angle) {
    println("Setting angle to " + new_Angle);
    wristInterface.setAngle(new_Angle);
  }

  public Angle getAngle() {
    return values.currentAngle;
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

  public void goUp(Angle offsetDegrees) {
    wristInterface.goUp(offsetDegrees);
  }

  public void goDown(Angle offsetDegrees) {
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
    SmartDashboard.putNumber("Subsystem/WristSubsystem/Gains/P", WristConstants.gains.kP());
    SmartDashboard.putNumber("Subsystem/WristSubsystem/Gains/I", WristConstants.gains.kI());
    SmartDashboard.putNumber("Subsystem/WristSubsystem/Gains/D", WristConstants.gains.kD());
    SmartDashboard.putBoolean("Commands/WristSetAngle/OverrideAngle", false);
    SmartDashboard.putNumber("Commands/WristSetAngle/ForcedAngle", 110);
  }

  @Override
  public void updateDashboard() {
    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    double newkP = SmartDashboard.getNumber("Subsystem/WristSubsystem/Gains/P", values.kP);
    double newkI = SmartDashboard.getNumber("Subsystem/WristSubsystem/Gains/I", values.kI);
    double newkD = SmartDashboard.getNumber("Subsystem/WristSubsystem/Gains/D", values.kD);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kP = CS_Utils.updateFromSmartDashboard(newkP, values.kP, (value) -> setkP(value));
    values.kI = CS_Utils.updateFromSmartDashboard(newkI, values.kI, (value) -> setkI(value));
    values.kD = CS_Utils.updateFromSmartDashboard(newkD, values.kD, (value) -> setkD(value));

    // Update the SmartDashboard with the current state of the subsystem
    SmartDashboard.putBoolean("Subsystem/WristSubsystem/Enabled", values.isEnabled);
    SmartDashboard.putNumber(
        "Subsystem/WristSubsystem/CurrentAngleDegrees", values.currentAngle.in(Degrees));
    SmartDashboard.putNumber("Subsystem/WristSubsystem/Amps", values.amps.in(Amps));
    SmartDashboard.putNumber(
        "Subsystem/WristSubsystem/DesiredAngleDegrees", values.desiredAngle.in(Degrees));
    SmartDashboard.putNumber("Subsystem/WristSubsystem/AppliedOutput", values.appliedOutput);
    SmartDashboard.putNumber(
        "Subsystem/WristSubsystem/Temperature", values.temperature.in(Celsius));

    SmartDashboard.putData(this);
    //   double newAngle =
    //       SmartDashboard.getNumber(
    //           "Subsystem/WristSubsystem/desiredAngleDegrees",
    // WristConstants.defaultAngleDegrees);
    //   if (newAngle != desiredAngleDegrees) {
    //     setAngleDegrees(newAngle);
    //   }
    //   SmartDashboard.putNumber("Subsystem/WristSubsystem/desiredAngleDegrees",
    // desiredAngleDegrees);
  }

  // Characterization methods
  public void runCharacterization(double input) {
    wristInterface.runCharacterization(input);
  }

  public double getCharacterizationAngle() {
    return values.currentAngle.in(Degrees);
  }
}
