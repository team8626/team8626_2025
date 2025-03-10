// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.commands.setters.units.ClimberSetAngle;

public class ToDoClimb extends ParallelCommandGroup {

  public ToDoClimb() {
    // ClimberSubsystem climber = ClimberSubsystem.climber;

    System.out.println("[Cmd: TODOCLIMB]");

    addCommands(
        Commodore.getSetStateCommand(CommodoreState.CLIMB_NOW), new ClimberSetAngle(() -> 180));
  }
}
