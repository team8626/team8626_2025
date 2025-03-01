// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.AlgaeShooterIntake;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.presets.Presets;
import java.util.function.DoubleSupplier;

public class ToAlgaeIntake extends SequentialCommandGroup {
  private AlgaeShooterSubsystem algae501;
  private DoubleSupplier rpmSupplier = null;

  public ToAlgaeIntake(DoubleSupplier newRMP) {
    this.algae501 = RobotContainer.algae501;
    rpmSupplier = newRMP;
    buildCommandGroup();
  }

  public ToAlgaeIntake() {
    this.algae501 = RobotContainer.algae501;
    System.out.println("[Cmd: TOALGAEINTAKE]");

    rpmSupplier = () -> Presets.ALGAE_FLOOR.getRPM();
    buildCommandGroup();
  }

  private void buildCommandGroup() {
    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                Commodore.getSetStateCommand(CommodoreState.ALGAE_INTAKE),
                new AlgaeShooterIntake(rpmSupplier)),
            new InstantCommand(), // empty command
            () -> !algae501.isLoaded()),
        new ConditionalCommand(
            Commodore.getSetStateCommand(CommodoreState.ALGAE_LOADED),
            Commodore.getSetStateCommand(CommodoreState.IDLE),
            () -> algae501.isLoaded()));
  }
}
