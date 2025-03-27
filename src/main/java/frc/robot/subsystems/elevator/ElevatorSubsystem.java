package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.gains0;
import static frc.robot.subsystems.elevator.ElevatorConstants.gains1;

import com.revrobotics.spark.ClosedLoopSlot;
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

  public void setkP0(double newkP0) {
    elevatorInterface.setPID(newkP0, values.kI0, values.kD0, ClosedLoopSlot.kSlot0);
  }

  public void setkI0(double newkI0) {
    elevatorInterface.setPID(values.kP0, values.kI0, newkI0, ClosedLoopSlot.kSlot0);
  }

  public void setkD0(double newkD0) {
    elevatorInterface.setPID(values.kP0, values.kI0, newkD0, ClosedLoopSlot.kSlot0);
  }

  public void setkP1(double newkP1) {
    elevatorInterface.setPID(newkP1, values.kI1, values.kD1, ClosedLoopSlot.kSlot1);
  }

  public void setkI1(double newkI1) {
    elevatorInterface.setPID(values.kP1, values.kI1, newkI1, ClosedLoopSlot.kSlot1);
  }

  public void setkD1(double newkD1) {
    elevatorInterface.setPID(values.kP1, values.kI1, newkD1, ClosedLoopSlot.kSlot1);
  }

  public void setMaxOutput0(double newMax) {
    elevatorInterface.setOutputRange(values.minOutput0, newMax, ClosedLoopSlot.kSlot0);
  }

  public void setMinOutput0(double newMin) {
    elevatorInterface.setOutputRange(newMin, values.maxOutput0, ClosedLoopSlot.kSlot0);
  }

  public void setMaxOutput1(double newMax) {
    elevatorInterface.setOutputRange(values.minOutput1, newMax, ClosedLoopSlot.kSlot1);
  }

  public void setMinOutput1(double newMin) {
    elevatorInterface.setOutputRange(newMin, values.maxOutput1, ClosedLoopSlot.kSlot1);
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
    SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/Gains/Slot0(Up)/P", gains0.kP());
    SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/Gains/Slot0(Up)/I", gains0.kI());
    SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/Gains/Slot0(Up)/D", gains0.kD());
    SmartDashboard.putNumber(
        "Subsystem/ElevatorSubsystem/Gains/Slot0(Up)/minOutput", gains0.minOutput());
    SmartDashboard.putNumber(
        "Subsystem/ElevatorSubsystem/Gains/Slot0(Up)/maxOutput", gains0.maxOutput());
    SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/Gains/Slot1(Down)/P", gains1.kP());
    SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/Gains/Slot1(Down)/I", gains1.kI());
    SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/Gains/Slot1(Down)/D", gains1.kD());
    SmartDashboard.putNumber(
        "Subsystem/ElevatorSubsystem/Gains/Slot1(Down)/minOutput", gains1.minOutput());
    SmartDashboard.putNumber(
        "Subsystem/ElevatorSubsystem/Gains/Slot1(Down)/maxOutput", gains1.maxOutput());

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
    double newkP0 =
        SmartDashboard.getNumber("Subsystem/ElevatorSubsystem/Gains/Slot0(Up)/P", values.kP0);
    double newkI0 =
        SmartDashboard.getNumber("Subsystem/ElevatorSubsystem/Gains/Slot0(Up)/I", values.kI0);
    double newkD0 =
        SmartDashboard.getNumber("Subsystem/ElevatorSubsystem/Gains/Slot0(Up)/D", values.kD0);
    double newMinOutput0 =
        SmartDashboard.getNumber(
            "Subsystem/ElevatorSubsystem/Gains/Slot0(Up)/minOutput", values.minOutput0);
    double newMaxOutput0 =
        SmartDashboard.getNumber(
            "Subsystem/ElevatorSubsystem/Gains/Slot0(Up)/maxOutput", values.maxOutput0);

    double newkP1 =
        SmartDashboard.getNumber("Subsystem/ElevatorSubsystem/Gains/Slot1(Down)/P", values.kP1);
    double newkI1 =
        SmartDashboard.getNumber("Subsystem/ElevatorSubsystem/Gains/Slot1(Down)/I", values.kI1);
    double newkD1 =
        SmartDashboard.getNumber("Subsystem/ElevatorSubsystem/Gains/Slot1(Down)/D", values.kD1);
    double newMinOutput1 =
        SmartDashboard.getNumber(
            "Subsystem/ElevatorSubsystem/Gains/Slot1(Down)/minOutput", values.minOutput1);
    double newMaxOutput1 =
        SmartDashboard.getNumber(
            "Subsystem/ElevatorSubsystem/Gains/Slot1(Down)/maxOutput", values.maxOutput1);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kP0 = CS_Utils.updateFromSmartDashboard(newkP0, values.kP0, (value) -> setkP0(value));
    values.kI0 = CS_Utils.updateFromSmartDashboard(newkI0, values.kI0, (value) -> setkI0(value));
    values.kD0 = CS_Utils.updateFromSmartDashboard(newkD0, values.kD0, (value) -> setkD0(value));
    values.minOutput0 =
        CS_Utils.updateFromSmartDashboard(
            newMinOutput0, values.minOutput0, (value) -> setMinOutput0(value));
    values.maxOutput0 =
        CS_Utils.updateFromSmartDashboard(
            newMaxOutput0, values.maxOutput0, (value) -> setMaxOutput0(value));

    values.kP1 = CS_Utils.updateFromSmartDashboard(newkP1, values.kP1, (value) -> setkP1(value));
    values.kI1 = CS_Utils.updateFromSmartDashboard(newkI1, values.kI1, (value) -> setkI1(value));
    values.kD1 = CS_Utils.updateFromSmartDashboard(newkD1, values.kD1, (value) -> setkD1(value));
    values.minOutput1 =
        CS_Utils.updateFromSmartDashboard(
            newMinOutput1, values.minOutput1, (value) -> setMinOutput1(value));
    values.maxOutput1 =
        CS_Utils.updateFromSmartDashboard(
            newMaxOutput1, values.maxOutput1, (value) -> setMaxOutput1(value));

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
