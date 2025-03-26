package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.gains;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public void setkP(double newkP) {
    elevatorInterface.setPID(newkP, values.kI, values.kD);
  }

  public void setkI(double newkI) {
    elevatorInterface.setPID(values.kP, values.kI, newkI);
  }

  public void setkD(double newkD) {
    elevatorInterface.setPID(values.kP, values.kI, newkD);
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

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kP = CS_Utils.updateFromSmartDashboard(newkP, values.kP, (value) -> setkP(value));
    values.kI = CS_Utils.updateFromSmartDashboard(newkI, values.kI, (value) -> setkI(value));
    values.kD = CS_Utils.updateFromSmartDashboard(newkD, values.kD, (value) -> setkD(value));

    SmartDashboard.putData(this);

    // double newHeight = SmartDashboard.getNumber("Subsystem/ElevatorSubsystem/DesiredHeight",
    // desiredHeight);
    // if (newHeight != desiredHeight) {
    //   desiredHeight = newHeight;
    //   setHeight(newHeight);
    // }
    // SmartDashboard.putNumber("Subsystem/ElevatorSubsystem/DesiredHeight", desiredHeight);
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
  // private void log(SysIdRoutineLog log) {
  //   System.out.println("logging");
  //   log.motor("left")
  //       .voltage(Voltage.ofRelativeUnits(left`Motor.get() * 12, Volts))
  //       .linearPosition(Distance.ofRelativeUnits(leftMotor.getEncoder().getPosition(), Inches))
  //       .linearVelocity(LinearVelocity.ofRelativeUnits(leftMotor.getEncoder().getVelocity(),
  // InchesPerSecond));
  //   log.motor("right")
  //       .voltage(Voltage.ofRelativeUnits(rightMotor.get() * 12, Volts))
  //       .linearPosition(Distance.ofRelativeUnits(rightMotor.getEncoder().getPosition(), Inches))
  //       .linearVelocity(LinearVelocity.ofRelativeUnits(rightMotor.getEncoder().getVelocity(),
  // InchesPerSecond));
  // }

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second),
              Volts.of(1.5),
              null,
              state -> SignalLogger.writeString("SysId_Elevator_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> elevatorInterface.setVoltageMainMotor(output.in(Volts)), null, this));

  private double getRotations() {
    return values.positionRight; // getPosition().in(Rotations);
  }

  private double getVelocity() {
    // return motor1.getVelocity().getValue().in(RotationsPerSecond);
    return values.velocityRight.in(InchesPerSecond);
  }

  public double getVolts() {
    return values.voltageRight;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return this.sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return this.sysIdRoutine.dynamic(direction);
  }
}
