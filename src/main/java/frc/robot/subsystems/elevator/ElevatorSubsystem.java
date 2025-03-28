package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.gains;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    this.setHeight(ElevatorConstants.minHeightInches);
    println("Created");
  }

  public void setHeight(double heightInches) {
    printf("New Height: %f (%f)", heightInches, getHeight());
    elevatorInterface.setHeightInches(heightInches);
  }

  public double getHeight() {
    return elevatorInterface.getHeightInches();
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
    SmartDashboard.putNumber("Subsystem/Elevator/Gains/P", gains.kP());
    SmartDashboard.putNumber("Subsystem/Elevator/Gains/I", gains.kI());
    SmartDashboard.putNumber("Subsystem/Elevator/Gains/D", gains.kD());
    SmartDashboard.putBoolean("Commands/ElevatorSetHeight/OverrideHeight", false);
    SmartDashboard.putNumber("Commands/ElevatorSetHeight/ForcedHeight", 50);
  }

  @Override
  public void updateDashboard() {
    // Update the SmartDashboard with the current state of the subsystem
    SmartDashboard.putNumber("Subsystem/Elevator/DesiredHeight", values.desiredHeight);
    SmartDashboard.putBoolean("Subsystem/Elevator/Enabled", values.isEnabled);
    SmartDashboard.putString("Subsystem/Elevator/State", values.state.getString());
    SmartDashboard.putNumber("Subsystem/Elevator/Height", values.currentHeight);
    SmartDashboard.putNumber("Subsystem/Elevator/AmpsLeft", values.ampsLeft);
    SmartDashboard.putNumber("Subsystem/Elevator/AmpsRight", values.ampsRight);
    SmartDashboard.putNumber("Subsystem/Elevator/AppliedOutputLeft", values.appliedOutputLeft);
    SmartDashboard.putNumber("Subsystem/Elevator/AppliedOutputRight", values.appliedOutputRight);
    SmartDashboard.putNumber("Subsystem/Elevator/TemperatureLeft", values.temperatureLeft);
    SmartDashboard.putNumber("Subsystem/Elevator/TemperatureRight", values.temperatureRight);
    SmartDashboard.putBoolean("Subsystem/Elevator/IsZeroed", values.isZeroed);

    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    double newkP = SmartDashboard.getNumber("Subsystem/Elevator/Gains/P", values.kP);
    double newkI = SmartDashboard.getNumber("Subsystem/Elevator/Gains/I", values.kI);
    double newkD = SmartDashboard.getNumber("Subsystem/Elevator/Gains/D", values.kD);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kP = CS_Utils.updateFromSmartDashboard(newkP, values.kP, (value) -> setkP(value));
    values.kI = CS_Utils.updateFromSmartDashboard(newkI, values.kI, (value) -> setkI(value));
    values.kD = CS_Utils.updateFromSmartDashboard(newkD, values.kD, (value) -> setkD(value));

    SmartDashboard.putData(this);

    // double newHeight = SmartDashboard.getNumber("Subsystem/Elevator/DesiredHeight",
    // desiredHeight);
    // if (newHeight != desiredHeight) {
    //   desiredHeight = newHeight;
    //   setHeight(newHeight);
    // }
    // SmartDashboard.putNumber("Subsystem/Elevator/DesiredHeight", desiredHeight);
  }

  @Override
  public void simulationPeriodic() {
    elevatorInterface.updateInputs(values);
  }

  public double getCharacterization() {
    return values.currentHeight;
  }

  public void goUp(double offsetInches) {
    elevatorInterface.goUp(offsetInches);
  }

  public void goDown(double offsetInches) {
    elevatorInterface.goDown(offsetInches);
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

}
