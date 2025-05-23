package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.gains;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.CS_SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorInterface.ElevatorValues;
import frc.utils.CS_Utils;

public class ElevatorSubsystem extends CS_SubsystemBase {
  private ElevatorInterface elevatorInterface;
  private ElevatorValues values;

  public ElevatorSubsystem(ElevatorInterface subsystem_interface) {
    super();

    this.elevatorInterface = subsystem_interface;
    values = new ElevatorValues();
    this.setHeight(ElevatorConstants.minHeight);
    println("Created");
  }

  public void setHeight(Distance height) {
    printf("New Height: %f (%f)", height.in(Inches), getHeight().in(Inches));
    elevatorInterface.setHeight(height);
  }

  public Distance getHeight() {
    return elevatorInterface.getHeight();
  }

  public void reset() {
    elevatorInterface.reset();
  }

  private void setkP(double newkP) {
    elevatorInterface.setPID(newkP, values.kI, values.kD);
  }

  private void setkI(double newkI) {
    elevatorInterface.setPID(values.kP, values.kI, newkI);
  }

  private void setkD(double newkD) {
    elevatorInterface.setPID(values.kP, values.kI, newkD);
  }

  private void setMaxOutput(double newMax) {
    elevatorInterface.outputRange(values.maxOutput, newMax);
  }

  private void setMinOutput(double newMin) {
    elevatorInterface.outputRange(newMin, values.maxOutput);
  }

  @Override
  public void CS_periodic() {
    elevatorInterface.updateInputs(values);
  }

  @Override
  public void initDashboard() {
    println("Initializing Dashboard");

    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/Gains/P", gains.kP());
    SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/Gains/I", gains.kI());
    SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/Gains/D", gains.kD());
    SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/Gains/Output/Min", gains.minOutput());
    SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/Gains/Output/Max", gains.maxOutput());

    SmartDashboard.putBoolean("Commands/ElevatorSetHeight/OverrideHeight", false);
    SmartDashboard.putNumber("Commands/ElevatorSetHeight/ForcedHeight", 50);
  }

  @Override
  public void updateDashboard() {
    // Update the SmartDashboard with the current state of the subsystem
    SmartDashboard.putNumber(
        "Subsystem/ElevatorSubsystem/DesiredHeight", values.desiredHeight.in(Inches));
    SmartDashboard.putBoolean("Subsystem/ElevatorSubsystem/Enabled", values.isEnabled);
    SmartDashboard.putString("Subsystem/ElevatorSubsystem/State", values.state.getString());
    SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/Height", values.currentHeight.in(Inches));
    SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/AmpsLeft", values.ampsLeft.in(Amps));
    SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/AmpsRight", values.ampsRight.in(Amps));
    SmartDashboard.putNumber(
        "Subsystem/ElevatorSubsystem/AppliedOutputLeft", values.appliedOutputLeft);
    SmartDashboard.putNumber(
        "Subsystem/ElevatorSubsystem/AppliedOutputRight", values.appliedOutputRight);
    SmartDashboard.putNumber(
        "Subsystem/ElevatorSubsystem/TemperatureLeft", values.temperatureLeft.in(Celsius));
    SmartDashboard.putNumber(
        "Subsystem/ElevatorSubsystem/TemperatureRight", values.temperatureRight.in(Celsius));
    SmartDashboard.putBoolean("Subsystem/ElevatorSubsystem/IsZeroed", values.isZeroed);

    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    double newkP = SmartDashboard.getNumber("Subsystem/ElevatorSubsystem/Gains/P", values.kP);
    double newkI = SmartDashboard.getNumber("Subsystem/ElevatorSubsystem/Gains/I", values.kI);
    double newkD = SmartDashboard.getNumber("Subsystem/ElevatorSubsystem/Gains/D", values.kD);
    double newMin =
        SmartDashboard.getNumber("Subsystem/ElevatorSubsystem/Gains/Output/Min", values.minOutput);
    double newMax =
        SmartDashboard.getNumber("Subsystem/ElevatorSubsystem/Gains/Output/Max", values.maxOutput);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kP = CS_Utils.updateFromSmartDashboard(newkP, values.kP, (value) -> setkP(value));
    values.kI = CS_Utils.updateFromSmartDashboard(newkI, values.kI, (value) -> setkI(value));
    values.kD = CS_Utils.updateFromSmartDashboard(newkD, values.kD, (value) -> setkD(value));
    values.minOutput =
        CS_Utils.updateFromSmartDashboard(newMin, values.minOutput, (value) -> setMinOutput(value));
    values.maxOutput =
        CS_Utils.updateFromSmartDashboard(newMax, values.maxOutput, (value) -> setMaxOutput(value));

    SmartDashboard.putData(this);
  }

  @Override
  public void simulationPeriodic() {
    elevatorInterface.updateInputs(values);
  }

  public double getCharacterization() {
    return values.currentHeight.in(Meters);
  }

  public void goUp(Distance offset) {
    elevatorInterface.goUp(offset);
  }

  public void goDown(Distance offset) {
    elevatorInterface.goDown(offset);
  }

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(7), null, null),
          // state -> SignalLogger.writeString("SysId_Elevator_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> elevatorInterface.setVoltageMainMotor(output), this::log, this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return this.sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return this.sysIdRoutine.dynamic(direction);
  }

  private void log(SysIdRoutineLog log) {
    log.motor("elevator-right")
        .voltage(values.voltsRight)
        .linearPosition(values.absolutePositionRight)
        .linearVelocity(values.currentVelocity);
  }
}
