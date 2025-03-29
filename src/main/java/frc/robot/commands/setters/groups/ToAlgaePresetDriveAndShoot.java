// Copyright (c) 2025 FRC 8626
// http://github.com/team8626
//
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.setters.groups;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.AlgaeShooterLaunch;
import frc.robot.commands.setters.units.AlgaeShooterRampUp;
import frc.robot.commands.setters.units.AlgaeShooterStop;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.presets.AlgaePreset;
import frc.robot.subsystems.presets.Presets;
import java.util.function.Supplier;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;

public class ToAlgaePresetDriveAndShoot extends SequentialCommandGroup {
  private final Timer timer = new Timer();
  private Supplier<AlgaePreset> algaePreset = null;

  // Presets are made using Blue reference, Flip them if needed.
  Supplier<Pose2d> targetPose = () -> AllianceFlipUtil.apply(algaePreset.get().getPose());
  Supplier<Pose2d> offsetPose =
      () ->
          AllianceFlipUtil.apply(
              algaePreset
                  .get()
                  .getPose()
                  .plus(
                      new Transform2d(
                          Units.inchesToMeters(-6), Units.inchesToMeters(-6), new Rotation2d())));

  private AlgaeShooterSubsystem algae501 = RobotContainer.algae501;

  public ToAlgaePresetDriveAndShoot(Supplier<AlgaePreset> newPreset) {
    algaePreset = newPreset;

    addCommands(
        new SequentialCommandGroup(
            // Drive to Offset Pose
            new SequentialCommandGroup(
                    Commodore.getSetStateCommand(CommodoreState.DRIVE_AUTO),
                    AutoBuilder.pathfindToPose(
                        offsetPose.get(), RobotConstants.PATH_CONSTRAINTS, 0.5),
                    Commodore.getSetStateCommand(CommodoreState.IDLE))
                .onlyIf(() -> !targetPose.get().equals(new Pose2d())),
            Commodore.getSetStateCommand(CommodoreState.IDLE),

            // Set Subsystems and Shoot
            new ParallelCommandGroup(
                // Drive to Target Pose

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
                    "Subsystem/AlgaeShooterSubsystem/LastShotIn(ms)", (int) (elapsedTime * 1000));
                Dashboard.publishAlgaeShootTime((int) (elapsedTime * 1000));
              }
            },
            new ToSubsystemsPreset(() -> Presets.ALGAE_STOW),
            Commodore.getSetStateCommand(CommodoreState.IDLE)));
  }
}
