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
import frc.robot.subsystems.coralshooter.CoralShooterConstants;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;

public class CoralShooterRampUp extends CS_Command {
  private CoralShooterSubsystem mortar;

  private double desiredRPMLeft = CoralShooterConstants.RPMShootLeft;
  private double desiredRPMRight = CoralShooterConstants.RPMShoolLeft;
  private final double RPMTolerance = CoralShooterConstants.RPMTolerance;
  private final double RPMDifferentialTolerance = CoralShooterConstants.RPMDifferentialTolerance;

  public CoralShooterRampUp() {
    mortar = RobotContainer.mortar;

    addRequirements(mortar);

    this.setTAGString("CORALSHOOTER_RAMPUP");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Dashboard.setCoralState(GamePieceState.RAMPING_UP);
    mortar.startRampUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      mortar.stopAll();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atSetpoint = false;

    double currentRPMLeft = mortar.getShooterRPMLeft();
    double currentRPMRight = mortar.getShooterRPMRight();

    atSetpoint =
        Math.abs(currentRPMLeft - desiredRPMLeft) <= RPMTolerance
            && Math.abs(currentRPMRight - desiredRPMLeft) <= RPMTolerance;

    atSetpoint =
        Math.abs(currentRPMLeft - desiredRPMLeft) <= RPMTolerance
            && Math.abs(currentRPMRight - desiredRPMRight) <= RPMTolerance
            && Math.abs(currentRPMLeft - currentRPMRight) <= RPMDifferentialTolerance;

    return atSetpoint;
  }
}
