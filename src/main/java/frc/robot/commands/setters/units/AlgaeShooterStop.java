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
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;

public class AlgaeShooterStop extends CS_Command {
  private AlgaeShooterSubsystem algae501;

  public AlgaeShooterStop() {
    // Use addRequirements() here to declare subsystem dependencies.
    // For example: addRequirements(Robot.m_subsystem);
    algae501 = RobotContainer.algae501;

    addRequirements(algae501);

    this.setTAGString("ALGAESHOOTER_STOP");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algae501.stopShooter();
    algae501.stopLauncher();
    Dashboard.setAlgaeState(GamePieceState.IDLE);
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
