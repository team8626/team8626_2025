package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.CS_InterfaceBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates.ElevatorState;

public class Elevator_Simulation implements ElevatorInterface, CS_InterfaceBase {

  private boolean is_enabled = false;
  private ElevatorState current_state = ElevatorState.STOPPED;
  private ElevatorSim elevator;
  private DCMotor elevatorGearbox = DCMotor.getNEO(1);

  public Elevator_Simulation() {
    
    elevator = new ElevatorSim(
          elevatorGearbox,
          ElevatorConstants.gearRatio,
          ElevatorConstants.carriageMassKg,
          ElevatorConstants.drumRadiusMeters,
          ElevatorConstants.minHeightMeters,
          ElevatorConstants.maxHeightMeters,
          true,
          0.0,
          new double[]{0.0, 0.01});
    }

  @Override
  public void updateInputs(ElevatorValues values) {
    values.current_height = getElevatorHeight();
    values.state = current_state;
    values.is_enabled = is_enabled;
  }

  @Override
  public void stopElevator() {
    setElevatorSpeed(0);
  }

  @Override
  public void setElevatorSpeed(double new_speed) {
    elevator.setInput(new_speed);
    if(new_speed > 0) {
      current_state = ElevatorState.MOVINGUP;
    } else if(new_speed < 0) {
      current_state = ElevatorState.MOVINGDOWN;
    } else {
      current_state = ElevatorState.STOPPED;
    }
    is_enabled = (new_speed != 0);
  }

  @Override
  public double getElevatorHeight() {
    return Units.metersToInches(elevator.getPositionMeters());
  }

  @Override
  public void setElevatorkP(double kP) {
    printf("New kP: %f\n", kP);
  }

  @Override
  public void setElevatorkI(double kI) {
    printf("New kI: %f\n", kI);
  }

  @Override
  public void setElevatorkD(double kD) {
    printf("New kD: %f\n", kD);
  }

  @Override
  public void setElevatorFF(double ff) {
    printf("New FF: %f\n", ff);
  }
}
