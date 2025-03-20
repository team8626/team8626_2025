// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.AlgaeShooterIntake;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ToAlgaeIntake extends SequentialCommandGroup {
  private AlgaeShooterSubsystem algae501;
  private DoubleSupplier rpmSupplier = null;

  public ToAlgaeIntake(DoubleSupplier newRMP) {
    this.algae501 = RobotContainer.algae501;
    rpmSupplier = newRMP;
    addCommands(new AlgaeShooterIntake(rpmSupplier).onlyIf(() -> !algae501.isLoaded()));
  }
}
