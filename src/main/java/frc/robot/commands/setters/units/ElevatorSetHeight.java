// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class ElevatorSetHeight extends CS_Command {
  private ElevatorSubsystem elevator;
  private DoubleSupplier height;

  public ElevatorSetHeight(DoubleSupplier newHeight) {
    elevator = RobotContainer.elevator;
    height = newHeight;

    addRequirements(elevator);
    SmartDashboard.putBoolean("Commands/ElevatorSetHeight/atSetPoint", false);

    this.setTAGString("ELEVATOR_SETHEIGHT");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setHeight(height.getAsDouble());
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
    boolean atSetpoint = false;

    double current_height = elevator.getHeight();

    if (Math.abs(current_height - height.getAsDouble()) <= ElevatorConstants.toleranceInches) {
      atSetpoint = true;
    }
    SmartDashboard.putBoolean("Commands/ElevatorSetHeight/atSetPoint", atSetpoint);

    return atSetpoint;
  }
}
