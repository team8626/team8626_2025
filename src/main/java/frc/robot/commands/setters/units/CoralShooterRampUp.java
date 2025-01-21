// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;

public class CoralShooterRampUp extends CS_Command {
  private CoralShooterSubsystem mortar;

  private double desiredRPM = CoralShooterConstants.shooterRPM;
  private final double RPMTolerance = CoralShooterConstants.shooterRPMTolerance;

  public CoralShooterRampUp() {
    mortar = RobotContainer.mortar;

    addRequirements(mortar);

    this.setTAGString("CORALSHOOTER_RAMPUP");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mortar.startShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      mortar.stopAll();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atSetpoint = false;

    double currentRPMLeft = mortar.getShooterRPMLeft();
    double currentRPMRight = mortar.getShooterRPMRight();

    atSetpoint = Math.abs(currentRPMLeft - desiredRPM) <= RPMTolerance && Math.abs(currentRPMRight - desiredRPM) <= RPMTolerance;

    return atSetpoint;
  }
}
