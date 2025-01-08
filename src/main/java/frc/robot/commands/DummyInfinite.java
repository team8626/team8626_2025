// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.dummy.DummySubsystem;

public class DummyInfinite extends CS_Command {
  private final Timer timer = new Timer();
  private double lastPrintTime = 0;

  private DummySubsystem dummy = RobotContainer.dummy;

  public DummyInfinite() {
    // Use addRequirements() here to declare subsystem dependencies.
    // For example: addRequirements(Robot.m_subsystem);
    dummy = RobotContainer.dummy;

    addRequirements(dummy);

    this.setTAGString("DUMMYINFINITE");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    lastPrintTime = 0;
    println("Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = timer.get();
    if (currentTime - lastPrintTime >= 1.0) {
      println("Elapsed time: " + (int) currentTime + " seconds");
      lastPrintTime = currentTime;
    }
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
    return false;
  }
}
