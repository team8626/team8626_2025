// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.AlgaeShooterLaunch;
import frc.robot.commands.setters.units.AlgaeShooterRampUp;
import frc.robot.commands.setters.units.AlgaeShooterStop;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.presets.AlgaePreset;
import frc.robot.subsystems.presets.Presets;
import java.util.function.Supplier;

public class ToAlgaePresetAndShoot extends SequentialCommandGroup {
  private final Timer timer = new Timer();
  private Supplier<AlgaePreset> algaePreset = null;
  private AlgaeShooterSubsystem algae501 = RobotContainer.algae501;

  public ToAlgaePresetAndShoot(Supplier<AlgaePreset> newPreset) {
    algaePreset = newPreset;

    addCommands(
        new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new ToSubsystemsPreset(() -> algaePreset.get()),
                    new WaitCommand(0.1),
                    new AlgaeShooterRampUp(() -> algaePreset.get().getRPM()) {
                      @Override
                      public void initialize() {
                        super.initialize();
                        timer.reset();
                        timer.start();
                      }
                    }),
                new WaitCommand(0.1),

                // Shoot Algae
                new AlgaeShooterLaunch(),

                // Stop Shooter and Reset
                new AlgaeShooterStop() {
                  @Override
                  public void initialize() {
                    super.initialize();
                    double elapsedTime = timer.get();
                    SmartDashboard.putNumber(
                        "Subsystem/AlgaeShooter/LastShotIn(ms)", (int) (elapsedTime * 1000));
                    Dashboard.publishAlgaeShootTime((int) (elapsedTime * 1000));
                  }
                },
                new ToSubsystemsPreset(() -> Presets.ALGAE_STOW))
            .finallyDo(
                interrupted -> {
                  new ParallelCommandGroup(
                          new InstantCommand(algae501::stopAll),
                          new ToSubsystemsPreset(() -> Presets.ALGAE_STOW),
                          Commodore.getSetStateCommand(CommodoreState.IDLE))
                      .schedule();
                }));
  }
}
