package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.wrist.WristConstants.wristConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.CS_InterfaceBase;

public class Wrist_Sim implements WristInterface, CS_InterfaceBase {

  private Angle desiredAngle = WristConstants.restAngle;
  // TODO: move these to contants
  // Gearing is 25:1 then a 24:60
  SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNeoVortex(1),
          wristConfig.reduction(),
          0.07318977,
          wristConfig.armLength().in(Meters),
          WristConstants.minAngle.in(Radians),
          WristConstants.maxAngle.in(Radians),
          false,
          WristConstants.restAngle.in(Radians),
          new double[0]);

  PIDController pidController = new PIDController(0.1, 0, 0);

  private boolean climberIsEnabled = false;

  public Wrist_Sim() {}

  @Override
  public void updateInputs(WristValues values) {
    desiredAngle =
        Degrees.of(
            MathUtil.clamp(
                desiredAngle.in(Degrees),
                WristConstants.minAngle.in(Degrees),
                WristConstants.maxAngle.in(Degrees)));
    this.pidController.setSetpoint(this.desiredAngle.in(Radians));
    double output = this.pidController.calculate(armSim.getAngleRads());
    this.armSim.setInput(MathUtil.clamp(output, -13, 13)); // Clamping on Batttery Voltage
    this.armSim.update(0.020);

    values.isEnabled = climberIsEnabled;
    values.currentAngle = getAngle();
    values.desiredAngle = this.desiredAngle;
    values.amps = Amps.of(this.armSim.getCurrentDrawAmps());
  }

  public Angle getAngle() {
    return Radians.of(armSim.getAngleRads());
  }

  @Override
  public void setAngle(Angle new_angle) {
    this.desiredAngle = new_angle;
    printf("New Angle %f", desiredAngle);
  }

  @Override
  public void setPID(double newkP, double newkI, double newkD) {
    printf("New PID: %f, %f, %f \n", newkP, newkI, newkD);
  }

  @Override
  public void goUp(Angle offset) {
    desiredAngle.minus(offset);
  }

  @Override
  public void goDown(Angle offset) {
    desiredAngle.plus(offset);
  }
}
