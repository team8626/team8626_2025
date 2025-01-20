// Copyright (c) 2024 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.setters.units.CoralShooterLaunch;
import frc.robot.commands.setters.units.CoralShooterStart;
import frc.robot.commands.setters.units.CoralShooterStop;

public class ToShootCoral extends SequentialCommandGroup {

  public ToShootCoral() {
    System.out.println("[Cmd: TOCORALSHOOT]");
    addCommands(
        new CoralShooterStart(),
        new CoralShooterLaunch(),
        new CoralShooterStop());
  }
}
