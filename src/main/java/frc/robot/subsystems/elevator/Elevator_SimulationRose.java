package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.CS_InterfaceBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates.ElevatorState;

public class Elevator_SimulationRose implements ElevatorInterface, CS_InterfaceBase {

  private boolean is_enabled = false;
  private ElevatorState current_state = ElevatorState.IDLE;
  private Distance desiredHeight = Meters.of(0);
  private Distance currentHeight = Meters.of(0);

  public Elevator_SimulationRose() {}

  private void updateRoseSim() {
    if (currentHeight.lt(desiredHeight)) {
      currentHeight.plus(Inches.of(0.5));
    } else if (currentHeight.gt(desiredHeight)) {
      currentHeight.minus(Inches.of(0.5));
      if (currentHeight.lt(desiredHeight)) {
        currentHeight.plus(Inches.of(0.5));
      } else if (currentHeight.gt(desiredHeight)) {
        currentHeight.minus(Inches.of(0.5));
      }
      desiredHeight =
          Meters.of(
              MathUtil.clamp(
                  desiredHeight.in(Meters),
                  ElevatorConstants.minHeight.in(Meters),
                  ElevatorConstants.maxHeight.in(Meters)));
      desiredHeight =
          Meters.of(
              MathUtil.clamp(
                  desiredHeight.in(Meters),
                  ElevatorConstants.minHeight.in(Meters),
                  ElevatorConstants.maxHeight.in(Meters)));
    }
  }

  @Override
  public void updateInputs(ElevatorValues values) {
    updateRoseSim();
    values.currentHeight = getHeight();
    values.desiredHeight = desiredHeight;
    values.currentHeight = getHeight();
    values.desiredHeight = desiredHeight;
    values.state = current_state;
    values.isEnabled = is_enabled;
  }

  @Override
  public Distance getHeight() {
    return (currentHeight);
  }

  @Override
  public void setHeight(Distance new_height) {
    desiredHeight = new_height;
    println("Setting height to: " + new_height.in(Inches));
  }

  @Override
  public void goUp(Distance offset) {
    desiredHeight.plus(offset);
  }

  @Override
  public void goDown(Distance offset) {
    desiredHeight.minus(offset);
  }

  @Override
  public void reset() {}

  @Override
  public void setVoltageMainMotor(Voltage voltage) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltageMainMotor'");
  }
}
