package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.CS_InterfaceBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates.ElevatorState;

public class Elevator_SimulationRose implements ElevatorInterface, CS_InterfaceBase {

  private boolean is_enabled = false;
  private ElevatorState current_state = ElevatorState.IDLE;
  private double desiredHeight = 0;
  private double currentHeight = 0;

  public Elevator_SimulationRose() {}

  private void updateRoseSim() {
    MathUtil.clamp(
        currentHeight, ElevatorConstants.minHeightInches, ElevatorConstants.maxHeightInches);
    if (currentHeight < desiredHeight) {
      currentHeight += 0.25;
    } else if (currentHeight > desiredHeight) {
      currentHeight -= 0.25;
    }
  }

  @Override
  public void updateInputs(ElevatorValues values) {
    updateRoseSim();
    values.currentHeight = getHeightInches();
    values.state = current_state;
    values.isEnabled = is_enabled;
  }

  @Override
  public double getHeightInches() {
    return (currentHeight);
  }

  @Override
  public void setElevatorkP(double new_value) {
    printf("New kP: %f\n", new_value);
  }

  @Override
  public void setElevatorkI(double new_value) {
    printf("New kI: %f\n", new_value);
  }

  @Override
  public void setElevatorkD(double new_value) {
    printf("New kD: %f\n", new_value);
  }

  @Override
  public void setHeightInches(double heightInches) {
    desiredHeight = heightInches;
    println("Setting height to: " + heightInches);
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
}
