// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Dashboard.GamePieceState;
import frc.robot.subsystems.algaeshooter.AlgaeShooterConstants;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import java.util.function.DoubleSupplier;

public class AlgaeShooterRampUp extends CS_Command {
  private AlgaeShooterSubsystem algae501;

  private double desiredRPM = AlgaeShooterConstants.shootRPM;
  private final double RPMTolerance = AlgaeShooterConstants.shooterRPMTolerance;

  // public AlgaeShooterRampUp() {
  //   algae501 = RobotContainer.algae501;

  //   addRequirements(algae501);

  //   this.desiredRPM = AlgaeShooterConstants.shootRPM;
  //   this.setTAGString("ALGAESHOOTER_RAMPUP");
  // }

  public AlgaeShooterRampUp(DoubleSupplier newRPM) {
    algae501 = RobotContainer.algae501;

    addRequirements(algae501);

    this.desiredRPM = newRPM.getAsDouble();
    this.setTAGString("ALGAESHOOTER_RAMPUP");
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    printf("RPM: %f", desiredRPM);

    Dashboard.setAlgaeState(GamePieceState.RAMPING_UP);
    algae501.startRampUp(desiredRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      algae501.stopAll();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atSetpoint = false;

    double currentRPMLeft = algae501.getShooterRPMLeft();
    double currentRPMRight = algae501.getShooterRPMRight();

    atSetpoint =
        Math.abs(Math.abs(currentRPMLeft) - desiredRPM) <= RPMTolerance
            && Math.abs(Math.abs(currentRPMRight) - desiredRPM) <= RPMTolerance;

    return atSetpoint;
  }
}
