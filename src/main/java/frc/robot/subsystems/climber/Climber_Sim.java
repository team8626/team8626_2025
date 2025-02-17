package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.armConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.CS_InterfaceBase;

public class Climber_Sim implements ClimberInterface, CS_InterfaceBase {

  private double goalDegrees;
  // TODO: move these to contants
  // Gearing is 25:1 then a 24:60
  SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          armConfig.reduction(),
          0.07318977,
          Units.inchesToMeters(armConfig.armLengthInches()),
          Units.degreesToRadians(ClimberConstants.minAngleDegrees),
          Units.degreesToRadians(ClimberConstants.maxAngleDegrees),
          false,
          Units.degreesToRadians(ClimberConstants.restAngleDegrees),
          new double[0]);

  PIDController pidController = new PIDController(0.1, 0, 0);

  private boolean climberIsEnabled = false;

  public Climber_Sim() {}

  @Override
  public void updateInputs(ClimberValues values) {
    double output = this.pidController.calculate(Units.radiansToDegrees(armSim.getAngleRads()));
    this.armSim.setInput(MathUtil.clamp(output, -13, 13)); // Clamping on Batttery Voltage
    this.armSim.update(0.020);

    values.climberIsEnabled = climberIsEnabled;
    values.currentAngleDegrees = getAngleDegrees();
    values.amps = this.armSim.getCurrentDrawAmps();
  }

  public double getAngleDegrees() {
    return Units.radiansToDegrees(armSim.getAngleRads());
  }

  // added to fix error at top of class (im hope this doesn't break anything)
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
