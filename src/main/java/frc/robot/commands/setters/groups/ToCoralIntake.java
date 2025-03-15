// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.setters.units.CoralShooterIntake2;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;

public class ToCoralIntake extends SequentialCommandGroup {

  public ToCoralIntake() {
    CoralShooterSubsystem mortar = RobotContainer.mortar;

    System.out.println("[Cmd: TOCORALINTAKE]");

    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                Commodore.getSetStateCommand(CommodoreState.CORAL_INTAKE),
                // new CoralShooterIntake()),
                new CoralShooterIntake2()),
            new ParallelCommandGroup(
                new RumbleCommand(), Commodore.getSetStateCommand(CommodoreState.IDLE)),
            () -> !mortar.isLoaded()),
        Commodore.getSetStateCommand(CommodoreState.IDLE));
  }
}
