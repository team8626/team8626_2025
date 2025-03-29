// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.CoralShooterLaunch;
import frc.robot.commands.setters.units.CoralShooterRampUp;
import frc.robot.commands.setters.units.CoralShooterStop;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.presets.CoralPreset;
import java.util.function.Supplier;

public class ToCoralShoot extends SequentialCommandGroup {
  private final Timer timer = new Timer();
  private CoralShooterSubsystem mortar;
  Supplier<CoralPreset> coralPreset = () -> new CoralPreset("empty");

  // TODO: Remove this constructor and ass preset in all calls (See Autos)
  public ToCoralShoot() {
    this.mortar = RobotContainer.mortar;
    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                new CoralShooterRampUp() {
                  @Override
                  public void initialize() {
                    super.initialize();
                    timer.reset();
                    timer.start();
                  }
                },
                new CoralShooterLaunch(),
                new CoralShooterStop() {
                  @Override
                  public void initialize() {
                    super.initialize();
                    double elapsedTime = timer.get();
                    SmartDashboard.putNumber(
                        "Subsystem/CoralShooterSubsystem/LastShotIn(ms)",
                        (int) (elapsedTime * 1000));
                    Dashboard.publishCoralShootTime((int) (elapsedTime * 1000));
                  }
                },
                Commodore.getSetStateCommand(CommodoreState.IDLE)
                // ,new InstantCommand(() -> PresetManager.resetCoralPreset())

                ),
            new SequentialCommandGroup(Commodore.getSetStateCommand(CommodoreState.IDLE)),
            mortar::isLoaded));
  }

  public ToCoralShoot(Supplier<CoralPreset> newCoralPreset) {
    this.mortar = RobotContainer.mortar;
    coralPreset = newCoralPreset;

    addCommands(
        new CoralShooterRampUp(coralPreset) {
          @Override
          public void initialize() {
            super.initialize();
            timer.reset();
            timer.start();
          }
        },
        new CoralShooterLaunch(),
        new CoralShooterStop() {
          @Override
          public void initialize() {
            super.initialize();
            double elapsedTime = timer.get();
            SmartDashboard.putNumber(
                "Subsystem/CoralShooterSubsystem/LastShotIn(ms)", (int) (elapsedTime * 1000));
            Dashboard.publishCoralShootTime((int) (elapsedTime * 1000));
          }
        },
        Commodore.getSetStateCommand(CommodoreState.IDLE));
  }
}
