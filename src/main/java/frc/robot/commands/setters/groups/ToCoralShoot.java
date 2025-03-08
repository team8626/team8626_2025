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
import frc.robot.subsystems.coralshooter.CoralShooterSubsystem;
import frc.robot.subsystems.presets.CoralPreset;
import frc.robot.subsystems.presets.PresetManager;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ToCoralShoot extends SequentialCommandGroup {
  private final Timer timer = new Timer();
  private CoralShooterSubsystem mortar;
  private DoubleSupplier rpmLeftSupplier = null;
  private DoubleSupplier rpmRightSupplier = null;

  public ToCoralShoot(Supplier<CoralPreset> presetSupplier) {
    this.mortar = RobotContainer.mortar;
    rpmLeftSupplier = () -> presetSupplier.get().getRPMLeft();
    rpmRightSupplier = () -> presetSupplier.get().getRPMRight();
    buildCommandGroup();
  }

  // public ToCoralShoot() {
  //   this.mortar = RobotContainer.mortar;
  //   System.out.println("[Cmd: TOALGAEINTAKE]");

  //   rpmLeftSupplier = () -> Presets.CORAL_L4.getRPMLeft();
  //   rpmRightSupplier = () -> Presets.CORAL_L4.getRPMRight();

  //   buildCommandGroup();
  // }

  private void buildCommandGroup() {
    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                Commodore.getSetStateCommand(CommodoreState.CORAL_SHOOT_RAMPINGUP),
                new CoralShooterRampUp(rpmLeftSupplier, rpmRightSupplier) {
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
                        "Subsystem/CoralShooter/Last Shot in (ms)", (int) (elapsedTime * 1000));
                  }
                },
                Commodore.getSetStateCommand(CommodoreState.IDLE),
                PresetManager.resetCoralPresetCmd()),
            new SequentialCommandGroup(Commodore.getSetStateCommand(CommodoreState.IDLE)),
            mortar::isLoaded));
  }
}
