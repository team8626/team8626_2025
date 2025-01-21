// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.CoralShooterLaunch;
import frc.robot.commands.setters.units.CoralShooterRampUp;
import frc.robot.commands.setters.units.CoralShooterStop;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;

public class ToCoralShoot extends SequentialCommandGroup {

  public ToCoralShoot() {
    CoralShooterSubsystem mortar = RobotContainer.mortar;

    System.out.println("[Cmd: TOCORALSHOOT]");
    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                Commodore.getSetStateCommand(CommodoreState.CORAL_SHOOT_RAMPINGUP),
                new CoralShooterRampUp(),
                Commodore.getSetStateCommand(CommodoreState.CORAL_SHOOT_LAUNCHING),
                new CoralShooterLaunch(),
                new CoralShooterStop(),
                Commodore.getSetStateCommand(CommodoreState.IDLE)),
            Commodore.getSetStateCommand(CommodoreState.IDLE),
            mortar::isLoaded));
  }
}
