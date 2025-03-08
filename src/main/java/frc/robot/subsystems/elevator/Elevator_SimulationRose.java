package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.CS_InterfaceBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates.ElevatorState;

public class Elevator_SimulationRose implements ElevatorInterface, CS_InterfaceBase {

  private boolean is_enabled = false;
  private ElevatorState current_state = ElevatorState.IDLE;
  private double desiredHeightInches = 0;
  private double currentHeight = 0;

  public Elevator_SimulationRose() {}

  private void updateRoseSim() {
    if (currentHeight < desiredHeightInches) {
      currentHeight += 0.5;
    } else if (currentHeight > desiredHeightInches) {
      currentHeight -= 0.5;
    }
    desiredHeightInches =
        MathUtil.clamp(
            desiredHeightInches,
            ElevatorConstants.minHeightInches,
            ElevatorConstants.maxHeightInches);
  }

  @Override
  public void updateInputs(ElevatorValues values) {
    updateRoseSim();
    values.currentHeight = getHeightInches();
    values.desiredHeight = desiredHeightInches;
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
    desiredHeightInches = heightInches;
    println("Setting height to: " + heightInches);
  }

  @Override
  public void goUp(double offsetInches) {
    desiredHeightInches += offsetInches;
  }

  @Override
  public void goDown(double offsetInches) {
    desiredHeightInches -= offsetInches;
  }

  @Override
  public void reset() {}
}
