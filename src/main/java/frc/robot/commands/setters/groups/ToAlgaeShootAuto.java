// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.AlgaeShooterLaunch;
import frc.robot.commands.setters.units.AlgaeShooterRampUp;
import frc.robot.commands.setters.units.AlgaeShooterStop;
import frc.robot.commands.setters.units.ElevatorSetHeight;
import frc.robot.commands.setters.units.WristSetAngle;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.presets.AlgaePreset;
import frc.robot.subsystems.presets.Presets;
import java.util.function.Supplier;

public class ToAlgaeShootAuto extends SequentialCommandGroup {
  private final Timer timer = new Timer();

  public ToAlgaeShootAuto(Supplier<AlgaePreset> presetSupplier) {
    AlgaeShooterSubsystem algae501 = RobotContainer.algae501;

    System.out.println("[Cmd: TOALGAESHOOT]");
    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new ElevatorSetHeight(() -> presetSupplier.get().getElevatorHeightInches()),
                    new WristSetAngle(() -> presetSupplier.get().getWristAngleDegrees())),
                Commodore.getSetStateCommand(CommodoreState.ALGAE_SHOOT_RAMPINGUP),
                new AlgaeShooterRampUp(() -> Presets.ALGAE_NETFROM10FT.getRPM()) {
                  @Override
                  public void initialize() {
                    super.initialize();
                    timer.reset();
                    timer.start();
                  }
                },
                Commodore.getSetStateCommand(CommodoreState.ALGAE_SHOOT_LAUNCHING),
                new AlgaeShooterLaunch(),
                new AlgaeShooterStop() {
                  @Override
                  public void initialize() {
                    super.initialize();
                    double elapsedTime = timer.get();
                    SmartDashboard.putNumber(
                        "Subsystem/AlgaeShooter/Last Shot in (ms)", (int) (elapsedTime * 1000));
                  }
                },
                Commodore.getSetStateCommand(CommodoreState.IDLE),
                new ToSubsystemsPreset(() -> Presets.ALGAE_STOW)),
            Commodore.getSetStateCommand(CommodoreState.IDLE),
            algae501::isLoaded));
  }
}
