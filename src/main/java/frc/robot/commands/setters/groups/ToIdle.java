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

public class ToIdle extends CS_Command {
  private CoralShooterSubsystem mortar = RobotContainer.mortar;

  public ToIdle() {
    // Add requirements to the subsystems that need to be reset when going back to idle
    // those subsystems will stop any ongoing action.
    if (mortar != null) {
      addRequirements(mortar);
    }
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
    // Check if subsystems are in a special state...
    if (mortar.isLoaded()) {
      Commodore.getSetStateCommand(CommodoreState.CORAL_LOADED).schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
