// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorMoveDown extends CS_Command {
  private ElevatorSubsystem elevator;
  private double previousHeight = 0;
  private double offset = 0.5;

  public ElevatorMoveDown() {
    elevator = RobotContainer.elevator;

    addRequirements(elevator);

    this.setTAGString("ELEVATOR_MOVEDOWN");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousHeight = elevator.getHeight();
    elevator.move(-1 * offset);
    printf(
        "ElevatorMoveDown:initialize() - Moving Down - Current: %f, (%f)\n",
        previousHeight, offset);
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
    boolean retVal = false;

    if (elevator.getHeight() < (previousHeight - offset)) {
      retVal = true;
    }
    return retVal;
  }
}
