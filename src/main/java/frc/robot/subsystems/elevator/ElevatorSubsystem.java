package frc.robot.subsystems.elevator;

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
    println("Created");
  }

  // Calls to the elevator interface
  public void stop() {
    elevatorInterface.stopElevator();
  }

  public void setSpeed(double new_height) {
    elevatorInterface.setElevatorSpeed(new_height);
  }

  public double getHeight() {
    return elevatorInterface.getElevatorHeight();
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

  public void setFF(double new_value) {
    elevatorInterface.setElevatorFF(new_value);
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
    SmartDashboard.putNumber("Subsystem/Elevator/P Gain", ElevatorConstants.kP);
    SmartDashboard.putNumber("Subsystem/Elevator/D Gain", ElevatorConstants.kI);
    SmartDashboard.putNumber("Subsystem/Elevator/I Gain", ElevatorConstants.kD);
    SmartDashboard.putNumber("Subsystem/Elevator/FF", ElevatorConstants.FF);
  }

  @Override
  public void updateDashboard() {
    // Update the SmartDashboard with the current state of the subsystem
    SmartDashboard.putBoolean("Subsystem/Elevator/Enabled", values.is_enabled);
    SmartDashboard.putString("Subsystem/Elevator/State", values.state.getString());
    SmartDashboard.putNumber("Subsystem/Elevator/Height", values.current_height);

    // SmartDashboard.putNumber("Subsystem/Elevator/P Gain", ElevatorConstants.kP);
    // SmartDashboard.putNumber("Subsystem/Elevator/D Gain", ElevatorConstants.kI);
    // SmartDashboard.putNumber("Subsystem/Elevator/I Gain", ElevatorConstants.kD);
    // SmartDashboard.putNumber("Subsystem/Elevator/FF Gain", ElevatorConstants.FF);

    // Using SmartDashboard to tune PIDs
    // --------------------------------------------------
    double newkP = SmartDashboard.getNumber("Subsystem/Elevator/P Gain", values.kP);
    double newkI = SmartDashboard.getNumber("Subsystem/Elevator/I Gain", values.kI);
    double newkD = SmartDashboard.getNumber("Subsystem/Elevator/D Gain", values.kD);
    double newFF = SmartDashboard.getNumber("Subsystem/Elevator/FF", values.FF);

    // Coefficients on SmartDashboard have changed, save new values to the PID controller
    // --------------------------------------------------
    values.kP = CS_Utils.updateFromSmartDashboard(newkP, values.kP, (value) -> setkP(value));
    values.kI = CS_Utils.updateFromSmartDashboard(newkI, values.kI, (value) -> setkI(value));
    values.kD = CS_Utils.updateFromSmartDashboard(newkD, values.kD, (value) -> setkD(value));
    values.FF = CS_Utils.updateFromSmartDashboard(newFF, values.FF, (value) -> setFF(value));

    // System.out.printf("P: %f, I: %f, D: %f, FF: %f\n", values.kP, values.kI, values.kD,
    // values.FF);
    // TODO Auto-generated method stub

  }

  @Override
  public void simulationPeriodic() {
    // TODO Auto-generated method stub

  }
}
