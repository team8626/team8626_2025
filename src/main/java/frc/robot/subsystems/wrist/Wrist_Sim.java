package frc.robot.subsystems.wrist;

import static frc.robot.subsystems.wrist.WristConstants.wristConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.CS_InterfaceBase;

public class Wrist_Sim implements WristInterface, CS_InterfaceBase {

  private double goalDegrees;
  // TODO: move these to contants
  // Gearing is 25:1 then a 24:60
  SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNeoVortex(1),
          wristConfig.reduction(),
          0.07318977,
          Units.inchesToMeters(wristConfig.armLengthInches()),
          Units.degreesToRadians(WristConstants.minAngleDegrees),
          Units.degreesToRadians(WristConstants.maxAngleDegrees),
          false,
          Units.degreesToRadians(WristConstants.restAngleDegrees),
          new double[0]);

  PIDController pidController = new PIDController(0.1, 0, 0);

  private boolean climberIsEnabled = false;

  public Wrist_Sim() {}

  @Override
  public void updateInputs(WristValues values) {
    double output = this.pidController.calculate(Units.radiansToDegrees(armSim.getAngleRads()));
    this.armSim.setInput(MathUtil.clamp(output, -13, 13)); // Clamping on Batttery Voltage
    this.armSim.update(0.020);

    values.isEnabled = climberIsEnabled;
    values.currentAngleDegrees = getAngleDegrees();
    values.amps = this.armSim.getCurrentDrawAmps();
  }

  public double getAngleDegrees() {
    return Units.radiansToDegrees(armSim.getAngleRads());
  }

  @Override
  public void setAngleDegrees(double new_angle) {
    printf("New Angle %f", new_angle);
    this.goalDegrees = new_angle;
    this.pidController.setSetpoint(new_angle);
  }

  @Override
  public void setPID(double newkP, double newkI, double newkD) {
    printf("New PID: %f, %f, %f \n", newkP, newkI, newkD);
  }
}
