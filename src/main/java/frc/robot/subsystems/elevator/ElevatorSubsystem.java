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

  public void setkP(double new_value) {
    elevatorInterface.setElevatorkP(new_value);
  }

  public void setkI(double new_value) {
    elevatorInterface.setElevatorkI(new_value);
  }

  public void setkD(double new_value) {
    elevatorInterface.setElevatorkD(new_value);
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
    SmartDashboard.putNumber("Subsystem/Elevator/P Gain", gains.kP());
    SmartDashboard.putNumber("Subsystem/Elevator/D Gain", gains.kI());
    SmartDashboard.putNumber("Subsystem/Elevator/I Gain", gains.kD());
  }

  @Override
  public void updateDashboard() {
    // Update the SmartDashboard with the current state of the subsystem
    SmartDashboard.putNumber("Subsystem/Elevator/DesiredHeight", values.desiredHeight);
    SmartDashboard.putBoolean("Subsystem/Elevator/Enabled", values.isEnabled);
    SmartDashboard.putString("Subsystem/Elevator/State", values.state.getString());
    SmartDashboard.putNumber("Subsystem/Elevator/Height", values.currentHeight);
    SmartDashboard.putNumber("Subsystem/Elevator/Amps", values.amps);
    SmartDashboard.putNumber("Subsystem/Elevator/Temperature", values.temperature);

    // SmartDashboard.putNumber("Subsystem/Elevator/P Gain", ElevatorConstants.kP);
    // SmartDashboard.putNumber("Subsystem/Elevator/D Gain", ElevatorConstants.kI);
    // SmartDashboard.putNumber("Subsystem/Elevator/I Gain", ElevatorConstants.kD);

    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    double newkP = SmartDashboard.getNumber("Subsystem/Elevator/P Gain", values.kP);
    double newkI = SmartDashboard.getNumber("Subsystem/Elevator/I Gain", values.kI);
    double newkD = SmartDashboard.getNumber("Subsystem/Elevator/D Gain", values.kD);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kP = CS_Utils.updateFromSmartDashboard(newkP, values.kP, (value) -> setkP(value));
    values.kI = CS_Utils.updateFromSmartDashboard(newkI, values.kI, (value) -> setkI(value));
    values.kD = CS_Utils.updateFromSmartDashboard(newkD, values.kD, (value) -> setkD(value));

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
}
