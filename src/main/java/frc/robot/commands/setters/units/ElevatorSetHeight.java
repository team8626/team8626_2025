// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.units;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.CS_Command;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.presets.Presets;
import java.util.function.Supplier;

public class ElevatorSetHeight extends CS_Command {
  private ElevatorSubsystem elevator;
  private Supplier<Distance> height;
  private boolean overrideHeight = false;
  private Distance dashboardHeight = Meters.of(0);
  private Distance desiredHeight = Presets.ALGAE_SHOOTBARGE_OURSIDE.getElevatorHeight();

  public ElevatorSetHeight(Supplier<Distance> newHeight) {
    elevator = RobotContainer.elevator;
    height = newHeight;

    addRequirements(elevator);
    SmartDashboard.putBoolean("Commands/ElevatorSetHeight/atSetPoint", false);

    this.setTAGString("ELEVATOR_SETHEIGHT");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredHeight = height.get();

    overrideHeight = SmartDashboard.getBoolean("Commands/ElevatorSetHeight/OverrideHeight", false);
    dashboardHeight =
        Inches.of(SmartDashboard.getNumber("Commands/ElevatorSetHeight/ForcedHeight", 0));
    if (overrideHeight) {
      desiredHeight = dashboardHeight;
    }
    elevator.setHeight(desiredHeight);
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

    Distance current_height = elevator.getHeight();

    if (current_height.isNear(height.get(), ElevatorConstants.tolerance)) {
      atSetpoint = true;
    }

    SmartDashboard.putBoolean("Commands/ElevatorSetHeight/atSetPoint", atSetpoint);

    return atSetpoint;
  }
}
