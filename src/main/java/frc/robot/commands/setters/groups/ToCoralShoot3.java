// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.CoralShooterLaunch;
import frc.robot.commands.setters.units.CoralShooterRampUp3;
import frc.robot.commands.setters.units.CoralShooterStop;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.presets.CoralPreset;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.Supplier;

public class ToCoralShoot3 extends SequentialCommandGroup {
  private final Timer timer = new Timer();
  private CoralShooterSubsystem mortar;

  public ToCoralShoot3() {
    this.mortar = RobotContainer.mortar;
    Supplier<CoralPreset> presetSupplier = () -> PresetManager.getCoralPreset();
    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                Commodore.getSetStateCommand(CommodoreState.CORAL_SHOOT_RAMPINGUP),
                // new CoralShooterRampUp3(() -> Presets.CORAL_L3) {
                new CoralShooterRampUp3() {
                  @Override
                  public void initialize() {
                    super.initialize();
                    timer.reset();
                    timer.start();
                  }
                },
                Commodore.getSetStateCommand(CommodoreState.CORAL_SHOOT_LAUNCHING),
                new CoralShooterLaunch(),
                new CoralShooterStop() {
                  @Override
                  public void initialize() {
                    super.initialize();
                    double elapsedTime = timer.get();
                    SmartDashboard.putNumber(
                        "Subsystem/CoralShooter/LastShotIn(ms)", (int) (elapsedTime * 1000));
                    Dashboard.publishCoralShootTime((int) (elapsedTime * 1000));
                  }
                },
                Commodore.getSetStateCommand(CommodoreState.IDLE),
                new PrintCommand("[TOCORALSOOT3] (From PresetManager) Resetting Coral Preset"),
                new InstantCommand(() -> PresetManager.resetCoralPreset())),
            new SequentialCommandGroup(Commodore.getSetStateCommand(CommodoreState.IDLE)),
            mortar::isLoaded));
  }

  public ToCoralShoot3(Supplier<CoralPreset> coralPresetSupplier) {
    this.mortar = RobotContainer.mortar;

    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                Commodore.getSetStateCommand(CommodoreState.CORAL_SHOOT_RAMPINGUP),
                new CoralShooterRampUp3(coralPresetSupplier) {
                  @Override
                  public void initialize() {
                    super.initialize();
                    timer.reset();
                    timer.start();
                  }
                },
                Commodore.getSetStateCommand(CommodoreState.CORAL_SHOOT_LAUNCHING),
                new CoralShooterLaunch(),
                new CoralShooterStop() {
                  @Override
                  public void initialize() {
                    super.initialize();
                    double elapsedTime = timer.get();
                    SmartDashboard.putNumber(
                        "Subsystem/CoralShooter/LastShotIn(ms)", (int) (elapsedTime * 1000));
                    Dashboard.publishCoralShootTime((int) (elapsedTime * 1000));
                  }
                },
                Commodore.getSetStateCommand(CommodoreState.IDLE),
                new InstantCommand(() -> PresetManager.resetCoralPreset())),
            new SequentialCommandGroup(Commodore.getSetStateCommand(CommodoreState.IDLE)),
            mortar::isLoaded));
  }
}
