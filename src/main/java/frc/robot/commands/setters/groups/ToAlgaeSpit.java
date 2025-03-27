// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.AlgaeShooterSpit;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.presets.Presets;
import java.util.function.Supplier;

public class ToAlgaeSpit extends SequentialCommandGroup {
  private AlgaeShooterSubsystem algae501;
  private Supplier<AngularVelocity> rpm = null;

  public ToAlgaeSpit(Supplier<AngularVelocity> newRMP) {
    this.algae501 = RobotContainer.algae501;
    rpm = newRMP;
    buildCommandGroup();
  }

  public ToAlgaeSpit() {
    this.algae501 = RobotContainer.algae501;

    rpm = () -> Presets.ALGAE_FLOOR.getRPM();
    buildCommandGroup();
  }

  private void buildCommandGroup() {
    addCommands(
        new ToSubsystemsPreset(() -> Presets.ALGAE_SPIT),
        new AlgaeShooterSpit().withTimeout(1.0),
        new ToSubsystemsPreset(() -> Presets.ALGAE_STOW),
        Commodore.getSetStateCommand(Commodore.CommodoreState.IDLE));
  }
}
