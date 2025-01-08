// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Commodore;
import frc.robot.RobotContainer;
import frc.robot.subsystems.dummy.DummySubsystem;

public class Dummy3sCommand extends CS_Command {
  private final Timer timer = new Timer();
  private DummySubsystem dummy = RobotContainer.dummy;

  public Dummy3sCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    // For example: addRequirements(Robot.m_subsystem);
    dummy = RobotContainer.dummy;

    addRequirements(dummy);

    this.setTAGString("DUMMY3S");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    println("Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // No specific action needed during execution

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    println("Ended" + (interrupted == true ? " (interrupted)" : ""));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(3.0) || Commodore.getCurrentState() == Commodore.CommodoreState.IDLE;
  }
}
