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
