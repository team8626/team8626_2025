// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.dummy.DummySubsystem;

public class ToIdle extends CS_Command {
  private DummySubsystem dummy = RobotContainer.dummy;
  private CoralShooterSubsystem mortar = RobotContainer.mortar;

  public ToIdle() {
    // Use addRequirements() here to declare subsystem dependencies.
    // For example: addRequirements(Robot.m_subsystem);
    // dummy = RobotContainer.dummy;
    // mortar = RobotContainer.mortar;

    addRequirements(mortar, dummy);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Commodore.getSetStateCommand(CommodoreState.IDLE).schedule();
    println("Ended" + (interrupted == true ? " (interrupted)" : ""));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
