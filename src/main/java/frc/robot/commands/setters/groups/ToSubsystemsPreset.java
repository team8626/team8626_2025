// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.commands.setters.units.ElevatorSetHeight;
import frc.robot.commands.setters.units.WristSetAngle;
import frc.robot.subsystems.presets.AlgaePreset;
import java.util.function.Supplier;

public class ToSubsystemsPreset extends ParallelCommandGroup {

  public ToSubsystemsPreset(Supplier<AlgaePreset> presetSupplier) {
    addCommands(
        Commodore.getSetStateCommand(CommodoreState.SUBSYSTEMS_ADJUST),
        new PrintCommand(
            "-------------------------\nSetting Subsystems - "
                + presetSupplier.get().getElevatorHeightInches()
                + " /  "
                + presetSupplier.get().getWristAngleDegrees()),
        new ElevatorSetHeight(() -> presetSupplier.get().getElevatorHeightInches()),
        new WristSetAngle(() -> presetSupplier.get().getWristAngleDegrees()),
        Commodore.getSetStateCommand(CommodoreState.SUBSYSTEMS_AT_SETPOINT),
        new PrintCommand("All SET!!!"));
  }
}
