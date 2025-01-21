// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;

public class CoralShooterStop extends CS_Command {
  private CoralShooterSubsystem mortar;

  public CoralShooterStop() {
    // Use addRequirements() here to declare subsystem dependencies.
    // For example: addRequirements(Robot.m_subsystem);
    mortar = RobotContainer.mortar;

    addRequirements(mortar);

    this.setTAGString("CORALSHOOTER_STOP");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mortar.stopShooter();
    mortar.stopLauncher();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
