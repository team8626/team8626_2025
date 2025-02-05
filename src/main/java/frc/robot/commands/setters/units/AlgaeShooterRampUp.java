// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.algaeshooter.AlgaeShooterConstants;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;

public class AlgaeShooterRampUp extends CS_Command {
  private AlgaeShooterSubsystem algae501;

  private double desiredRPM = AlgaeShooterConstants.shootRPM;
  private final double RPMTolerance = AlgaeShooterConstants.shooterRPMTolerance;

  public AlgaeShooterRampUp() {
    algae501 = RobotContainer.algae501;

    addRequirements(algae501);

    this.setTAGString("ALGAESHOOTER_RAMPUP");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algae501.startRampUp();
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
        Math.abs(currentRPMLeft - desiredRPM) <= RPMTolerance
            && Math.abs(currentRPMRight - desiredRPM) <= RPMTolerance;

    return atSetpoint;
  }
}
