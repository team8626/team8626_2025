package frc.robot.commands.setters.groups;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commodore;
import frc.robot.Commodore.CommodoreState;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.setters.units.DriveToPoseFinkle2;
import frc.robot.subsystems.algaeshooter.AlgaeShooterSubsystem;
import frc.robot.subsystems.presets.AlgaePreset;
import frc.robot.subsystems.presets.PresetManager;
import frc.robot.subsystems.presets.Presets;
import java.util.function.Supplier;

public class ToPathAndFinkleAndAlgaeShoot extends SequentialCommandGroup {
  private final Timer timer = new Timer();

  Supplier<Pose2d> targetPose = () -> PresetManager.getAlgaePreset().get().getShootingPose();
  Supplier<Pose2d> offsetPose =
      () ->
          PresetManager.getAlgaePreset()
              .get()
              .getShootingPose()
              .plus(
                  new Transform2d(
                      Units.inchesToMeters(-18), Units.inchesToMeters(-18), new Rotation2d()));
  private Supplier<AlgaePreset> algaePreset;
  AlgaeShooterSubsystem algae501 = RobotContainer.algae501;

  public ToPathAndFinkleAndAlgaeShoot(Supplier<AlgaePreset> presetSupplier) {
    algaePreset = presetSupplier;

    addCommands(
        new SequentialCommandGroup(
            // Drive to Offset Pose
            new SequentialCommandGroup(
                    Commodore.getSetStateCommand(CommodoreState.DRIVE_AUTO),
                    AutoBuilder.pathfindToPose(
                        offsetPose.get(), RobotConstants.PATH_CONSTRAINTS, 0.5),
                    Commodore.getSetStateCommand(CommodoreState.IDLE))
                .onlyIf(() -> !targetPose.get().equals(new Pose2d()))
                .finallyDo((interrupted) -> Commodore.setCommodoreState(CommodoreState.IDLE)),

            // Drive to Target Pose while setting subsystems
            new ParallelCommandGroup(
                // Drive to Target Pose
                new SequentialCommandGroup(new DriveToPoseFinkle2(targetPose))
                    .onlyIf(() -> !targetPose.get().equals(new Pose2d())),

                // Adjust Subsystems & Start Rampup
                new ToSubsystemsPreset(algaePreset)),

            // Shoot Algae
            new ToAlgaeShoot(algaePreset),

            // Reset subsystems
            new ToSubsystemsPreset(() -> Presets.ALGAE_STOW),
            Commodore.getSetStateCommand(CommodoreState.IDLE)));
  }
}
