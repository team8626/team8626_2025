package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CS_InterfaceBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates.ElevatorState;

public class Elevator_SimulationRose implements ElevatorInterface, CS_InterfaceBase {

  private boolean is_enabled = false;
  private ElevatorState current_state = ElevatorState.STOPPED;
  private double desiredHeight = 0;
  private double currentHeight = 0;

  public Elevator_SimulationRose() {}

  private void updateRoseSim() {
    MathUtil.clamp(
        currentHeight,
        Units.metersToInches(ElevatorConstants.minHeightMeters),
        Units.metersToInches(ElevatorConstants.maxHeightMeters));
    if (currentHeight < desiredHeight) {
      currentHeight += 0.25;
    } else if (currentHeight > desiredHeight) {
      currentHeight -= 0.25;
    }
  }

  @Override
  public void updateInputs(ElevatorValues values) {
    updateRoseSim();
    values.current_height = getHeightInches();
    values.state = current_state;
    values.is_enabled = is_enabled;
  }

  @Override
  public void stopElevator() {
    setElevatorSpeed(0);
  }

  @Override
  public void setElevatorSpeed(double new_speed) {}

  @Override
  public double getHeightInches() {
    return (currentHeight);
  }
  /**
   * Move the elevator by offset
   *
   * @param offsetInches
   */
  public void moveInches(double offsetInches) {}

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
  public void setElevatorFF(double new_value) {
    printf("New FF: %f\n", new_value);
  }

  @Override
  public void setHeightInches(double heightInches) {
    desiredHeight = heightInches;
    println("Setting height to: " + heightInches);
  }
}
