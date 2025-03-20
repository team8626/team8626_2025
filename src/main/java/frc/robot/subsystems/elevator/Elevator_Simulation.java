package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.CS_InterfaceBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates.ElevatorState;

public class Elevator_Simulation implements ElevatorInterface, CS_InterfaceBase {

  private boolean is_enabled = false;
  private ElevatorState current_state = ElevatorState.IDLE;
  private ElevatorSim elevator;
  private DCMotor elevatorGearbox = DCMotor.getNEO(1);

  public Elevator_Simulation() {

    elevator =
        new ElevatorSim(
            elevatorGearbox,
            ElevatorConstants.gearRatio,
            ElevatorConstants.carriageMassKg,
            Units.inchesToMeters(ElevatorConstants.drumRadiusInches),
            Units.inchesToMeters(ElevatorConstants.minHeightInches),
            Units.inchesToMeters(ElevatorConstants.maxHeightInches),
            true,
            0.0,
            new double[] {0.0, 0.01});
  }

  @Override
  public void updateInputs(ElevatorValues values) {
    elevator.update(0.02);
    values.currentHeight = getHeightInches();
    values.state = current_state;
    values.isEnabled = is_enabled;
  }

  @Override
  public double getHeightInches() {
    return Units.metersToInches(elevator.getPositionMeters());
  }
  /**
   * Move the elevator by offset
   *
   * @param offsetInches
   */
  public void moveInches(double offsetInches) {
    printf("Moving Elevator by %f inches\n", offsetInches);
    printf("Current Height: %f\n", getHeightInches());
    elevator.setState(
        Units.inchesToMeters(offsetInches) + this.getHeightInches(),
        ElevatorConstants.velocityMetersPerSecond);
  }

  @Override
  public void setHeightInches(double heightInches) {
    elevator.setState(
        Units.inchesToMeters(heightInches), ElevatorConstants.velocityMetersPerSecond);
  }

  @Override
  public void goUp(double offsetInches) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'goUp'");
  }

  @Override
  public void goDown(double offsetInches) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'goDown'");
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }
}
