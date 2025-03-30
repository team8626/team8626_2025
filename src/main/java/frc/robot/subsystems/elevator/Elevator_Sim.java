package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.gains0;
import static frc.robot.subsystems.elevator.ElevatorConstants.gains1;

import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.CS_InterfaceBase;
import frc.utils.CS_Utils;

public class Elevator_Sim implements ElevatorInterface, CS_InterfaceBase {

  private Distance desiredSetpoint =
      ElevatorConstants.initHeight.minus(ElevatorConstants.cascadingOffset);

  private ElevatorConstants.Gains pid0gains = gains0;
  private ElevatorConstants.Gains pid1gains = gains1;

  ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getNeoVortex(2),
          1,
          ElevatorConstants.carriageMass.in(Kilograms),
          ElevatorConstants.drumRadius.in(Meters),
          ElevatorConstants.minHeight.minus(ElevatorConstants.cascadingOffset).in(Meters),
          ElevatorConstants.maxHeight.minus(ElevatorConstants.cascadingOffset).in(Meters),
          false,
          ElevatorConstants.initHeight.minus(ElevatorConstants.cascadingOffset).in(Meters),
          new double[0]);

  PIDController pidController = new PIDController(pid0gains.kP(), pid0gains.kI(), pid0gains.kD());

  private boolean elevatorIsEnabled = false;

  private boolean isEnabled = false;
  private boolean isZeroed = true;

  public Elevator_Sim() {}

  @Override
  public void updateInputs(ElevatorValues values) {
    values.currentHeight = getHeight();
    values.currentVelocity = MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond());
    values.desiredHeight = this.desiredSetpoint.plus(ElevatorConstants.cascadingOffset);
    values.ampsLeft = Amps.of(elevatorSim.getCurrentDrawAmps());
    values.ampsRight = Amps.of(elevatorSim.getCurrentDrawAmps());
    values.voltsLeft = Volts.of(0);
    values.voltsRight = Volts.of(0);
    values.temperatureLeft = Celsius.of(0);
    values.temperatureRight = Celsius.of(0);
    values.appliedOutputLeft = 0;
    values.appliedOutputRight = 0;
    values.absolutePositionRight = Meters.of(elevatorSim.getPositionMeters());
    values.velocityRight = MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond());

    values.isEnabled = this.isEnabled;
    values.isZeroed = this.isZeroed;

    boolean goingUp = desiredSetpoint.gt(Meters.of(elevatorSim.getPositionMeters()));
    if (goingUp) {
      this.pidController.setPID(pid0gains.kP(), pid0gains.kI(), pid0gains.kD());
    } else {
      this.pidController.setPID(pid1gains.kP(), pid1gains.kI(), pid1gains.kD());
    }
    this.pidController.setSetpoint(desiredSetpoint.in(Meters));
    double output = this.pidController.calculate(elevatorSim.getPositionMeters());
    this.elevatorSim.setInput(MathUtil.clamp(output, -13, 13)); // Clamping on Batttery Voltage
    this.elevatorSim.update(0.020);
  }

  // Zero the elevator by srunning it reverse until it hits the bottom ---Gently---
  public void reset() {}

  @Override
  public Distance getHeight() {
    Distance retval =
        Meters.of(elevatorSim.getPositionMeters()).plus(ElevatorConstants.cascadingOffset);
    return retval;
  }

  @Override
  public void setOutputRange(double min, double max, ClosedLoopSlot slot) {
    printf("New Range %s: %f, %f, %f", slot.toString(), min, max);
  }

  @Override
  public void setPID(double newkP, double newkI, double newkD, ClosedLoopSlot slot) {
    printf("New PID %s: %f, %f, %f", slot.toString(), newkP, newkI, newkD);

    switch (slot) {
      case kSlot0:
        pid0gains = new ElevatorConstants.Gains(newkP, newkI, newkD);
        break;
      case kSlot1:
        pid1gains = new ElevatorConstants.Gains(newkP, newkI, newkD);
        break;
      default:
    }
    pidController.setPID(newkP, newkI, newkD);
  }

  @Override
  public void setHeight(Distance new_height) {
    // desiredSetpoint = new_height.minus(ElevatorConstants.cascadingOffset);

    desiredSetpoint =
        (CS_Utils.clamp(new_height, ElevatorConstants.minHeight, ElevatorConstants.maxHeight))
            .minus(ElevatorConstants.cascadingOffset);

    // printf(
    //     "Setting Height: %f [%f - %f]\n",
    //     desiredSetpoint.plus(ElevatorConstants.cascadingOffset).in(Inches),
    //     ElevatorConstants.minHeight.in(Inches),
    //     ElevatorConstants.maxHeight.in(Inches));
  }

  @Override
  public void goUp(Distance offset) {
    desiredSetpoint =
        CS_Utils.clamp(
                desiredSetpoint.plus(offset).plus(ElevatorConstants.cascadingOffset),
                ElevatorConstants.minHeight,
                ElevatorConstants.maxHeight)
            .minus(ElevatorConstants.cascadingOffset);
  }

  @Override
  public void goDown(Distance offset) {
    desiredSetpoint =
        CS_Utils.clamp(
                desiredSetpoint.minus(offset).plus(ElevatorConstants.cascadingOffset),
                ElevatorConstants.minHeight,
                ElevatorConstants.maxHeight)
            .minus(ElevatorConstants.cascadingOffset);
  }

  @Override
  public void setVoltageMainMotor(Voltage voltage) {
    // printf("Setting Voltage: %f\n", voltage.in(Volts));
  }
}
