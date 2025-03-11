// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setters.units.ElevatorSetHeight;
import frc.robot.commands.setters.units.WristSetAngle;
import frc.robot.subsystems.presets.AlgaePreset;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class ToDeAlgaefy extends SequentialCommandGroup {

  public ToDeAlgaefy(Supplier<AlgaePreset> presetSupplier) {
    System.out.println("[Cmd: TODEALGAE]");

    addCommands(
        new ParallelCommandGroup(
            new PrintCommand(
                "-------------------------\nStarting DeAlgaefy - "
                    + presetSupplier.get().getElevatorHeightInches()
                    + " /  "
                    + presetSupplier.get().getWristAngleDegrees()),
            new ElevatorSetHeight(() -> presetSupplier.get().getElevatorHeightInches()),
            new WristSetAngle(() -> presetSupplier.get().getWristAngleDegrees())),
        new ToAlgaeIntake(() -> presetSupplier.get().getRPM()),
        new InstantCommand(() -> PresetManager.resetAlgaePreset()));
  }
}
